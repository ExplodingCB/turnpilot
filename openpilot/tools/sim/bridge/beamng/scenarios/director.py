"""The game side of a scenario run: spawns, moves and removes the scenario's actors over its own BeamNGpy client, in a
thread of the bridge process, so no game round trip (about one game frame each) ever stalls the bridge's 100 Hz loop.

Jobs (any callable) run in the order submitted and return a Future. Between jobs the thread walks the mannequins, one
re-pose per round trip, and polls the scenario cars' poses at 5 Hz into `poses`, which the runner reads for gaps.

set_lights forces signal instances (a scenario's forced light) and logs each with the wall time it was set (light_log,
which chase_render.py replays); cleanup puts them back: green and held where the bridge holds every light green, else
back on their sequences.
"""
import queue
import threading
import time
from concurrent.futures import Future

import numpy as np
from beamngpy import BeamNGpy, Vehicle

from openpilot.tools.sim.bridge.beamng import scenario
from openpilot.tools.sim.bridge.beamng.scenario import GREEN, RED, YELLOW
from openpilot.tools.sim.bridge.beamng.scenarios.catalog import PREFIX, Actor, project, section
from openpilot.tools.sim.bridge.beamng.scenarios.timing import (arc_lengths, catch_up_lead, pacer_drop_s, point_at,
                                                                script_nodes, walker_pose)

CAR_MODEL = 'etk800'   # the ego's model, already loaded: no new assets in the PC's short RAM
POLL_PERIOD = 0.2      # s between pose polls of the scenario cars
DONE_AFTER_S = 3.      # a scripted car is removed this long after its script's last node
PACER_AHEAD_M = 2.     # the pacer settles this far ahead of the car's camera
Z_ABOVE = 3.           # ground ray casts start this far above the road graph's height


KNOWN = [f"{PREFIX}{n}" for n in ('ped', 'pacer', 'occluder', *(f'car{k}' for k in range(6)))]  # every name the catalog gives out
LIGHT_STATES = {'green': GREEN, 'yellow': YELLOW, 'red': RED}


def wall() -> float:
  return time.time()  # noqa: TID251  the light log is aligned with the pose logs on wall-clock time


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
    self.forced: set[str] = set()  # signal instances set_lights forced, until cleanup puts them back
    self.lights_green = True       # ...green and held (the bridge without --signals), else back on their sequences
    self.light_log: list[list] = []  # [wall, instance, state] of each set_lights
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
        self.cars[a.name] = {'actor': a, 'vehicle': v, 'end_t': None}
        self.poses[a.name] = (pos[0], pos[1], a.heading, pos[2])

  def walk(self, name: str):
    self.walkers[name]['t0'] = time.monotonic()

  def release(self, name: str):
    """Send an oncoming car down its lane: from standing, up to its speed."""
    car = self.cars[name]
    a = car['actor']
    self._script(car, script_nodes(a.path, a.speed, a.accel))

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

  def _script(self, car, nodes):
    v = car['vehicle']
    v.control(throttle=0.0, brake=0.0, parkingbrake=0.0)
    v.ai.set_script(nodes, cling=True)
    car['end_t'] = time.monotonic() + nodes[-1]['t'] + DONE_AFTER_S

  def set_lights(self, states: dict[str, str], green_held: bool):
    """Force each signal instance to its state ('red', 'yellow', 'green') and hold the sequences, which would overwrite a
    strict state. green_held: the bridge holds every light green (no --signals), what cleanup puts them back to."""
    self.lights_green = green_held
    for name, state in states.items():
      self.bng.traffic_signals.set_instance_strict_state(name, LIGHT_STATES[state])
      self.forced.add(name)
      self.light_log.append([round(wall(), 3), name, state])
    scenario.hold_signals(self.bng)

  def restore_lights(self):
    if not self.forced:
      return
    if self.lights_green:
      scenario.force_signals(self.bng, sorted(self.forced), GREEN)
    else:
      scenario.release_signals(self.bng, sorted(self.forced))
    self.forced.clear()

  def cleanup(self):
    """Remove every actor this run spawned (vehicles first: they hold the most game memory), and put back the lights it
    forced."""
    self.restore_lights()
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
        (x, y, z), h = scenario.ego_pose(self.bng, name)
        self.poses[name] = (x, y, h, z)
