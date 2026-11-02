"""Status for the phone: a thread reads cereal into a plain snapshot, and handlers format the latest one."""
import statistics
import threading
import time
from collections import deque

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.turnpilot.control_text import DECIDER_ACTIONS, calibration_text, hold_text, is_map_reason

SERVICES = ["deviceState", "navInstruction", "turnpilotPlan", "gpsLocation", "gpsLocationExternal",
            "modelV2", "turnpilotDetections", "turnpilotFarDetections",  # these three for the diagnostics' model and step times
            "extrinsicsCalibration",  # its calibration line
            "managerState"]  # and this for its GPS section: which GPS daemons run
GPS_SERVICES = ("gpsLocation", "gpsLocationExternal")
GPS_DAEMONS = ("ubloxd", "pigeond", "qcomgpsd")
TIME_SAMPLES = 20  # snapshots each step time's median is over (10 s at SNAPSHOT_PERIOD)
SNAPSHOT_PERIOD = 0.5  # s between snapshots; the page polls every 2 s, so 2 Hz is plenty
SERVICE_TIMEOUT = 3.  # s since a service last published before it counts as not running
STALE_AFTER = 5.  # s without a new snapshot means the reader thread is stuck or dead

STATE_LABELS = {
  "disabled": "Off", "noRoute": "No route", "following": "Following route", "approaching": "Approaching maneuver",
  "turning": "Turning", "laneChange": "Changing lanes", "holding": "Holding", "arrived": "Arrived",
  "unavailable": "Unavailable", "offline": "Not running",
}
MANEUVER_NAMES = {"new name": "continue", "end of road": "turn", "on ramp": "take ramp", "off ramp": "take exit",
                  "rotary": "roundabout", "roundabout turn": "roundabout"}


def service_up(sm, s) -> bool:
  """SubMaster's own alive is 10 periods, which is half a second for turnpilotPlan; the phone only needs a few seconds."""
  return bool(sm.seen[s]) and time.monotonic() - sm.recv_time[s] < SERVICE_TIMEOUT


def read_services(sm) -> dict:
  """Plain values out of a SubMaster. Nothing else in the web app touches cereal."""
  def up(s):
    return service_up(sm, s)

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
             "prompt": str(plan.controlPrompt), "alert": str(plan.controlAlert), "missed": plan.turnMissed,  # the diagnostics' events
             "handback": str(plan.handback),
             "map": {"kind": str(plan.mapControl.kind), "action": str(plan.mapControl.action), "light": str(plan.mapControl.lightState),
                     "source": str(plan.mapControl.lightSource), "status": str(plan.mapControl.controlsStatus)}}
            if up("turnpilotPlan") else None,
    "gps": (sm[source].latitude, sm[source].longitude) if source else None,
  }


def read_live(sm, samples: dict[str, deque]) -> dict:
  """The driving model and the detector as they run now, for the diagnostics: modelV2.big (the big model on the chestnut),
  nativeStatus and farStatus, each step time's median over the last TIME_SAMPLES snapshots that brought a new message, and
  the cameras' calibration (calStatus and calPerc; None while calibrationd isn't publishing), which the tracker waits for."""
  for s, key, read in (("modelV2", "model_s", lambda m: m.modelExecutionTime), ("turnpilotDetections", "full_s", lambda m: m.executionTime),
                       ("turnpilotFarDetections", "far_s", lambda m: m.run.executionTime)):
    if sm.updated[s]:
      samples.setdefault(key, deque(maxlen=TIME_SAMPLES)).append(float(read(sm[s])))
    elif not service_up(sm, s):
      samples.pop(key, None)  # stopped: no stale times
  td = sm["turnpilotDetections"] if service_up(sm, "turnpilotDetections") else None
  cal = sm["extrinsicsCalibration"] if service_up(sm, "extrinsicsCalibration") else None
  return {
    "model_big": bool(sm["modelV2"].big) if service_up(sm, "modelV2") else None,
    "native": str(td.nativeStatus) if td is not None else None,
    "far": str(td.farStatus) if td is not None else None,
    "calibration": (str(cal.calStatus), int(cal.calPerc)) if cal is not None else None,
    **{key: statistics.median(v) for key, v in samples.items() if v},
    "samples": len(samples.get("model_s", ())),
  }


def read_gps(sm) -> dict:
  """The diagnostics' GPS section, never a coordinate: each GPS daemon's managerState running (None: managerState isn't
  publishing), and for gpsLocation and gpsLocationExternal whether it publishes, valid, hasFix, the last message's age and
  the fields that say how good a fix is."""
  procs = {}
  if service_up(sm, "managerState"):
    running = {p.name: bool(p.running) for p in sm["managerState"].processes}
    procs = {d: running.get(d) for d in GPS_DAEMONS}
  services = {}
  for s in GPS_SERVICES:
    if not sm.seen[s]:
      services[s] = {"seen": False}
      continue
    g = sm[s]
    services[s] = {"seen": True, "up": service_up(sm, s), "valid": bool(sm.valid[s]), "fix": bool(g.hasFix),
                   "age_s": time.monotonic() - sm.recv_time[s], "source": str(g.source), "satellites": int(g.satelliteCount),
                   "h_acc": float(g.horizontalAccuracy), "v_acc": float(g.verticalAccuracy), "speed_acc": float(g.speedAccuracy),
                   "bearing_acc": float(g.bearingAccuracyDeg)}
  return {"procs": procs, "services": services}


def see_ublox(radio: dict, msgs) -> None:
  """The u-blox's own radio readings for the diagnostics, from every ubloxGnss message since the last snapshot: MON-HW's
  noise and AGC (interference raises the noise; no antenna signal at all leaves it low), its antenna and jamming state,
  and how many satellites the receiver actually hears (C/N0 above 0) against how many it expects. Never a position."""
  for m in msgs:
    g = m.ubloxGnss
    w = g.which()
    if w == "hwStatus":
      h = g.hwStatus
      radio["hw"] = {"noise": int(h.noisePerMS), "agc": int(h.agcCnt), "antenna": str(h.aStatus), "antenna_power": str(h.aPower),
                     "jamming": str(h.jamInd), "at": time.monotonic()}
    elif w == "satReport":
      svs = g.satReport.svs
      radio["sats"] = {"listed": len(svs), "heard": sum(1 for sv in svs if sv.cno > 0),
                       "best_cno": max((int(sv.cno) for sv in svs), default=0), "at": time.monotonic()}
    elif w == "measurementReport":
      radio["meas"] = {"count": int(g.measurementReport.numMeas), "at": time.monotonic()}


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
  decider = bool(m) and m["action"] in DECIDER_ACTIONS and m["kind"] in ("stop", "yield")  # the stop-sign decider's (stop_go.py)
  held = hold_text(m["kind"], m["action"], m["light"], m["source"], reason or "") if m and (plan["state"] == "holding" or decider) else None
  calibrating = calibration_text(reason or "", metric) if plan and plan["state"] == "unavailable" else None
  if held is not None and decider:  # its wait, creep, go and band by its reason, whatever the plan's state
    label, reason = held
  elif held is not None:  # stopped at a stop sign or signal ("Stopped at stop sign", "take control"), or another hold with it
    label, reason = held[0], held[1] if is_map_reason(reason) or not reason else reason
  elif calibrating is not None:  # waiting for the cameras' calibration, not a detector out ("Calibrating cameras", "45%, ...")
    label, reason = calibrating
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
  """events: the diagnostics' EventLog, fed each snapshot's plan and every errorLogMessage."""
  def __init__(self, events=None):
    super().__init__(name="status", daemon=True)
    self.lock = threading.Lock()
    self.snapshot = None
    self.stopped = threading.Event()
    self.events = events
    self.samples: dict[str, deque] = {}
    self.radio: dict = {}

  def stop(self):
    self.stopped.set()

  def get(self):
    with self.lock:
      return self.snapshot

  def run(self):
    sm, errors, ublox, logged = None, None, None, False
    while not self.stopped.is_set():
      try:
        if sm is None:
          # imported here so a missing messaging build leaves the search and destination working
          from openpilot.cereal import messaging
          sm = messaging.SubMaster(SERVICES)
          errors = messaging.sub_sock("errorLogMessage", conflate=False) if self.events is not None else None
          ublox = messaging.sub_sock("ubloxGnss", conflate=False)
        sm.update(0)  # the sockets conflate, so this takes the newest message of each service
        snap = read_services(sm)
        snap["live"] = read_live(sm, self.samples)
        snap["gps_diag"] = read_gps(sm)
        see_ublox(self.radio, messaging.drain_sock(ublox))
        snap["gps_diag"]["radio"] = dict(self.radio)
        with self.lock:
          self.snapshot = snap
        if self.events is not None:
          self.events.see_plan(snap["plan"])
          for msg in messaging.drain_sock(errors):
            self.events.see_log(msg.errorLogMessage)
      except Exception:
        if not logged:
          cloudlog.exception("turnpilot_web.status_failed")
          logged = True
        sm = None
        self.stopped.wait(5)
      self.stopped.wait(SNAPSHOT_PERIOD)
