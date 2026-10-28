"""Status for the phone: a thread reads cereal into a plain snapshot, and handlers format the latest one."""
import threading
import time

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.turnpilot.control_text import hold_text, is_map_reason

SERVICES = ["deviceState", "navInstruction", "turnpilotPlan", "gpsLocation", "gpsLocationExternal"]
SNAPSHOT_PERIOD = 0.5  # s between snapshots; the page polls every 2 s, so 2 Hz is plenty
SERVICE_TIMEOUT = 3.  # s since a service last published before it counts as not running
STALE_AFTER = 5.  # s without a new snapshot means the reader thread is stuck or dead

STATE_LABELS = {
  "disabled": "Off", "noRoute": "No route", "following": "Following route", "approaching": "Approaching maneuver",
  "turning": "Turning", "laneChange": "Changing lanes", "holding": "Holding", "arrived": "Arrived",
  "offline": "Not running",
}
MANEUVER_NAMES = {"new name": "continue", "end of road": "turn", "on ramp": "take ramp", "off ramp": "take exit",
                  "rotary": "roundabout", "roundabout turn": "roundabout"}


def read_services(sm) -> dict:
  """Plain values out of a SubMaster. Nothing else in the web app touches cereal."""
  def up(s):  # SubMaster's own alive is 10 periods, which is half a second for turnpilotPlan; the phone only needs a few seconds
    return sm.seen[s] and time.monotonic() - sm.recv_time[s] < SERVICE_TIMEOUT

  # navd sends navInstruction with valid=False when it has no route, and turnpilotd doesn't set valid at all
  ni, plan = sm["navInstruction"], sm["turnpilotPlan"]
  route = None
  if up("navInstruction") and sm.valid["navInstruction"]:
    route = {"text": ni.maneuverPrimaryText, "type": ni.maneuverType, "modifier": ni.maneuverModifier,
             "maneuver_m": ni.maneuverDistance, "remaining_m": ni.distanceRemaining, "remaining_s": ni.timeRemaining}
  source = latest_gps_source(sm)
  return {
    "at": time.monotonic(),
    "onroad": bool(up("deviceState") and sm["deviceState"].started),
    "route": route,
    "plan": {"state": str(plan.state), "reason": plan.reason, "shadow": plan.shadow,
             "map": {"kind": str(plan.mapControl.kind), "action": str(plan.mapControl.action), "light": str(plan.mapControl.lightState),
                     "source": str(plan.mapControl.lightSource), "status": str(plan.mapControl.controlsStatus)}}
            if up("turnpilotPlan") else None,
    "gps": (sm[source].latitude, sm[source].longitude) if source else None,
  }


def format_distance(meters: float, metric: bool) -> str:
  if metric:
    if meters < 1000:
      return f"{round(meters, -1):.0f} m"
    return f"{meters / 1000:.1f} km" if meters < 10000 else f"{meters / 1000:.0f} km"
  miles = meters / 1609.344
  if miles < 0.2:
    return f"{round(meters * 3.28084, -1):.0f} ft"
  return f"{miles:.1f} mi" if miles < 10 else f"{miles:.0f} mi"


def format_duration(seconds: float) -> str:
  minutes = round(seconds / 60)
  if minutes < 1:
    return "<1 min"
  return f"{minutes} min" if minutes < 60 else f"{minutes // 60} h {minutes % 60:02d} min"


def maneuver_text(kind: str, modifier: str, road: str) -> str:
  """'turn', 'left', 'Main St' -> 'Turn left onto Main St'."""
  if kind == "arrive":
    return f"Arrive at {road}" if road else "Arrive"
  words = " ".join(w for w in (MANEUVER_NAMES.get(kind, kind), modifier) if w).capitalize()
  return f"{words} onto {road}" if words and road else words or road


def offline_reason(fresh: bool, onroad: bool, enabled: bool) -> str:
  """Why there is no plan from turnpilotd. It only runs onroad, and publishes while enabled."""
  if not fresh:  # the snapshot is a placeholder, so onroad says nothing
    return "car data is not available"
  if not enabled:
    return "TurnPilot is off"
  return "turnpilotd is not running" if onroad else "car is off"


def format_status(snap: dict | None, destination, enabled: bool, metric: bool, now: float | None = None) -> dict:
  """The /api/status body. snap comes from read_services, destination from Store.destination."""
  now = time.monotonic() if now is None else now
  fresh = snap is not None and now - snap["at"] < STALE_AFTER
  if snap is None or not fresh:
    snap = {"onroad": False, "route": None, "plan": None, "gps": None}
  route, plan = snap["route"], snap["plan"]
  label = STATE_LABELS.get(plan["state"], plan["state"]) if plan else STATE_LABELS["offline"]
  reason = plan["reason"] if plan else offline_reason(fresh, snap["onroad"], enabled)
  m = (plan or {}).get("map")
  held = hold_text(m["kind"], m["action"], m["light"], m["source"]) if m and plan["state"] == "holding" else None
  if held is not None:  # stopped at a stop sign or signal ("Stopped at stop sign", "take control"), or another hold with it
    label, reason = held[0], held[1] if is_map_reason(reason) or not reason else reason
  return {
    "reading": fresh,
    "onroad": snap["onroad"],
    "destination": destination,
    "navigating": route is not None,
    "maneuver": None if route is None else {
      "text": maneuver_text(route["type"], route["modifier"], route["text"]),
      "distance": format_distance(route["maneuver_m"], metric),
    },
    "remaining": None if route is None else {
      "distance": format_distance(route["remaining_m"], metric),
      "duration": format_duration(route["remaining_s"]),
      "seconds": route["remaining_s"],  # the phone adds this to its own clock to get an arrival time
    },
    "turnpilot": {
      "enabled": enabled,
      "state": plan["state"] if plan else "offline",
      "label": label,
      "reason": reason,
      "shadow": plan["shadow"] if plan else False,
    },
    "gps": snap["gps"] and {"latitude": snap["gps"][0], "longitude": snap["gps"][1]},
  }


class StatusReader(threading.Thread):
  def __init__(self):
    super().__init__(name="status", daemon=True)
    self.lock = threading.Lock()
    self.snapshot = None
    self.stopped = threading.Event()

  def stop(self):
    self.stopped.set()

  def get(self):
    with self.lock:
      return self.snapshot

  def run(self):
    sm, logged = None, False
    while not self.stopped.is_set():
      try:
        if sm is None:
          # imported here so a missing messaging build leaves the search and destination working
          from openpilot.cereal import messaging
          sm = messaging.SubMaster(SERVICES)
        sm.update(0)  # the sockets conflate, so this takes the newest message of each service
        snap = read_services(sm)
        with self.lock:
          self.snapshot = snap
      except Exception:
        if not logged:
          cloudlog.exception("turnpilot_web.status_failed")
          logged = True
        sm = None
        self.stopped.wait(5)
      self.stopped.wait(SNAPSHOT_PERIOD)
