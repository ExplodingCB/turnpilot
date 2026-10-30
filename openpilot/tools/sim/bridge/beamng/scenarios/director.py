"""The game side of a scenario run: spawns, moves and removes the scenario's actors over its own BeamNGpy client, in a
thread of the bridge process, so no game round trip (about one game frame each) ever stalls the bridge's 100 Hz loop.

Jobs (any callable) run in the order submitted and return a Future. Between jobs the thread walks the mannequins, one
re-pose per round trip, and polls the scenario cars' poses at 5 Hz into `poses`, which the runner reads for gaps.
"""
import math
import queue
import threading
import time
from concurrent.futures import Future

import numpy as np
from beamngpy import BeamNGpy, Vehicle

from openpilot.tools.sim.bridge.beamng import scenario
from openpilot.tools.sim.bridge.beamng.scenarios.catalog import PREFIX, Actor, project, section
from openpilot.tools.sim.bridge.beamng.scenarios.timing import (arc_lengths, catch_up_lead, pacer_drop_s, point_at,
                                                                script_nodes, walker_pose)

CAR_MODEL = 'etk800'   # the ego's model, already loaded: no new assets in the PC's short RAM
POLL_PERIOD = 0.2      # s between pose polls of the scenario cars
DONE_AFTER_S = 3.      # a scripted car is removed this long after its script's last node
PARK_AFTER_S = 0.3     # a car driven up to its line is parked this long after its script's last node
STAND_S = 0.5          # ...and stands at least this long before its next leg
PACER_AHEAD_M = 2.     # the pacer settles this far ahead of the car's camera
Z_ABOVE = 3.           # ground ray casts start this far above the road graph's height


KNOWN = [f"{PREFIX}{n}" for n in ('ped', 'pacer', *(f'car{k}' for k in range(6)), *(f'cross{k}' for k in range(4)))]  # every catalog name


def sweep(bng, log=print) -> int:
  """Delete every vehicle and object named with PREFIX: what a crashed or killed run left behind."""
  n = 0
  for vid in list(bng.vehicles.get_current_info(include_config=False)):
    if vid.startswith(PREFIX):
      scenario.lua(bng, f"local o = scenetree.findObject('{vid}') if o then o:delete() end")
      n += 1
  # the known names first, by findObject as scenario.py does; then a scan of the mission group for anything else
  for name in KNOWN:
    r = scenario.lua(bng, f"local o = scenetree.findObject('{name}') if o then o:delete() return '1' end return '0'", response=True)
    n += str(r) == '1'
  r = scenario.lua(bng, "local ok, n = pcall(function() local g, n = scenetree.MissionGroup, 0 " +
                        "for i = g:getCount() - 1, 0, -1 do local o = g:getObject(i) " +
                        f"if o and string.find(o:getName() or '', '^{PREFIX}') then o:delete() n = n + 1 end end return n end) " +
                        "return tostring(ok and n or -1)", response=True)
  if str(r) == '-1':
    log("sweep: couldn't scan the mission group for mannequins")
  else:
    n += int(float(r))
  return n


class Director(threading.Thread):
  def __init__(self, host: str, port: int, log=print):
    super().__init__(name='scenario_director', daemon=True)
    self.host, self.port, self.log = host, port, log
    self.jobs: queue.Queue = queue.Queue()
    self.poses: dict[str, tuple] = {}  # name -> (x, y, heading, z), the latest
    self.walkers: dict[str, dict] = {}
    self.cars: dict[str, dict] = {}
    self.bng = None
    self.stopped = threading.Event()
    self._next_poll = 0.

  def submit(self, fn, *args) -> Future:
    f: Future = Future()
    self.jobs.put((f, fn, args))
    return f

  def close(self, timeout: float = 30.):
    """Remove everything this run spawned, disconnect and stop. Blocks."""
    self.submit(self.cleanup)
    self.submit(self.stopped.set)
    self.join(timeout)

  def run(self):
    try:
      self.bng = BeamNGpy(self.host, self.port).open(launch=False)
      while not self.stopped.is_set():
        try:
          f, fn, args = self.jobs.get(timeout=0.01 if self.walkers or self.cars else 0.1)
        except queue.Empty:
          pass
        else:
          try:
            f.set_result(fn(*args))
          except Exception as e:
            self.log(f"scenario job {getattr(fn, '__name__', fn)} failed: {e!r}")
            f.set_exception(e)
        self.tick()
    except Exception as e:
      self.log(f"scenario director stopped: {e!r}")
      while not self.jobs.empty():
        self.jobs.get()[0].set_exception(e)
    finally:
      if self.bng is not None:
        try:
          self.cleanup()
        except Exception as e:
          self.log(f"scenario cleanup failed: {e!r}")
        self.bng.disconnect()
      self.stopped.set()

  # *** jobs ***
  def setup(self, actors: list[Actor]):
    """Place this run's actors: mannequins at the start of their walk, cars standing at the start of their lane."""
    for a in actors:
      path = np.array(a.path, dtype=float)
      if a.kind == 'walker':
        for p in path:  # every corner of the walk on the ground: the road graph's z is the junction's
          z = scenario.ground_z(self.bng, p[0], p[1], p[2] + Z_ABOVE)
          p[2] = z if abs(z - p[2]) < Z_ABOVE else p[2]  # a missed ray reads 2 km down
        scenario.spawn_pedestrian(self.bng, a.name, *path[0], heading=a.heading)
        self.walkers[a.name] = {'actor': a, 'path': path, 't0': None, 'done': False}
        self.poses[a.name] = (float(path[0][0]), float(path[0][1]), a.heading, float(path[0][2]))
      else:
        v = Vehicle(a.name, model=CAR_MODEL, license='SCENARIO', color=(0.75, 0.75, 0.78))
        pos = (float(path[0][0]), float(path[0][1]), float(path[0][2]) + 0.5)
        if not self.bng.vehicles.spawn(v, pos, scenario.heading_quat(a.heading)):
          raise RuntimeError(f"couldn't spawn {a.name}")
        v.ai.set_mode('disabled')
        # hold on the parking brake: BeamNG's arcade automatic reads the brake pedal at a standstill as throttle in
        # reverse, so a car held on brake=1 backs away from the junction at full power before its release
        v.control(throttle=0.0, brake=0.0, parkingbrake=1.0)
        self.cars[a.name] = {'actor': a, 'vehicle': v, 'end_t': None, 'park_t': None, 'parked_t': None, 'pending': None}
        self.poses[a.name] = (pos[0], pos[1], a.heading, pos[2])

  def walk(self, name: str):
    self.walkers[name]['t0'] = time.monotonic()

  def release(self, name: str):
    """Send an oncoming car down its lane: from standing, up to its speed."""
    car = self.cars[name]
    a = car['actor']
    self._script(car, script_nodes(a.path, a.speed, a.accel))

  def arrive(self, name: str):
    """Drive a stop sign's cross car up to its line (its path: catalog.cross_actors) and stand it there, parked: from
    standing, up to its speed, and braking at its accel to a stop at the path's end."""
    car = self.cars[name]
    a = car['actor']
    length = float(arc_lengths(np.asarray(a.path, dtype=float))[-1])
    v_top = min(a.speed, math.sqrt(a.accel * length))  # half the way up, half down where it never gets to its speed
    self._script(car, script_nodes(a.path, a.speed, a.accel, length - v_top ** 2 / (2. * a.accel), 0.), stays=True)

  def cross_on(self, name: str):
    """Send a stop sign's cross car on from its line: across the junction (path2 after its arrive, else its path), from
    standing. One still driving up to its line goes once it has stood there STAND_S."""
    car = self.cars[name]
    a = car['actor']
    nodes = script_nodes(a.params.get('path2') or a.path, a.speed, a.accel)
    if car['park_t'] is not None or (car['parked_t'] is not None and time.monotonic() - car['parked_t'] < STAND_S):
      car['pending'] = nodes
    else:
      self._script(car, nodes)

  def release_pacer(self, name: str, ego_xy, v_ego: float):
    """Put the pacer in the target lane just ahead of the car, by as much as it loses getting up to speed, and send it
    along at the car's set speed until it drops back and clears the lane."""
    car = self.cars[name]
    a = car['actor']
    lane = np.array(a.path, dtype=float)
    s_ego, _, _ = project(lane, np.asarray(ego_xy, dtype=float))
    s0 = s_ego + catch_up_lead(v_ego, a.speed, a.accel) + PACER_AHEAD_M
    end = float(arc_lengths(lane)[-1])
    if s0 > end - 20.:
      raise RuntimeError("the pacer's lane ends before the car's position")
    p, h = point_at(lane, s0)
    self.bng.vehicles.teleport(name, (float(p[0]), float(p[1]), float(p[2]) + 0.5), scenario.heading_quat(h))
    drop = pacer_drop_s(s0, a.params['s_junction'], a.params['window_start_m'], a.speed, a.params['block_s'])
    self._script(car, script_nodes(section(lane, s0, end), a.speed, a.accel, drop, a.speed * a.params['drop_to']))

  def _script(self, car, nodes, stays: bool = False):
    """Run an AI script; the car is removed DONE_AFTER_S after its end, or (stays) parked PARK_AFTER_S after it."""
    v = car['vehicle']
    v.control(throttle=0.0, brake=0.0, parkingbrake=0.0)
    v.ai.set_script(nodes, cling=True)
    end = time.monotonic() + nodes[-1]['t']
    car['end_t'], car['park_t'] = (None, end + PARK_AFTER_S) if stays else (end + DONE_AFTER_S, None)
    car['parked_t'] = None

  def cleanup(self):
    """Remove every actor this run spawned (vehicles first: they hold the most game memory)."""
    for name, car in list(self.cars.items()):
      self._remove_car(name, car)
    for name in list(self.walkers):
      scenario.remove_pedestrian(self.bng, name)
      self.walkers.pop(name)
      self.poses.pop(name, None)

  def _remove_car(self, name, car):
    try:
      self.bng.vehicles.despawn(car['vehicle'])
    except Exception as e:  # it may already be gone; the sweep deletes by name anyway
      self.log(f"despawn {name} failed: {e!r}")
      scenario.lua(self.bng, f"local o = scenetree.findObject('{name}') if o then o:delete() end")
    self.cars.pop(name, None)
    self.poses.pop(name, None)

  # *** motion ***
  def tick(self):
    now = time.monotonic()
    for name, w in self.walkers.items():
      if w['t0'] is None or w['done']:
        continue
      t = now - w['t0']
      p, h = walker_pose(w['path'], t, w['actor'].speed)
      scenario.move_pedestrian(self.bng, name, float(p[0]), float(p[1]), float(p[2]), h)
      self.poses[name] = (float(p[0]), float(p[1]), h, float(p[2]))
      w['done'] = t * w['actor'].speed > float(arc_lengths(w['path'])[-1])
    if self.cars and now >= self._next_poll:
      self._next_poll = now + POLL_PERIOD
      for name, car in list(self.cars.items()):
        if car['end_t'] is not None and now > car['end_t']:
          self._remove_car(name, car)  # past the junction and gone: free the memory
          continue
        if car.get('park_t') is not None and now > car['park_t']:  # up at its line: parked (catalog.cross_actors)
          car['vehicle'].ai.set_mode('disabled')
          car['vehicle'].control(throttle=0.0, brake=0.0, parkingbrake=1.0)
          car['park_t'], car['parked_t'] = None, now
        if car.get('pending') is not None and car.get('parked_t') is not None and now - car['parked_t'] >= STAND_S:
          nodes, car['pending'] = car['pending'], None
          self._script(car, nodes)
        (x, y, z), h = scenario.ego_pose(self.bng, name)
        self.poses[name] = (x, y, h, z)
