"""BeamNG.tech as an openpilot sim World: one ego car on a stock map, driven over BeamNGpy from this Mac.

Connections (BeamNGpy connections are not thread safe, and BeamNG.tech accepts several clients):
  self.bng       game engine: scenario setup, scenario hooks and teardown, under bng_lock
  ego.connection the car: control and electrics, in the vehicle thread
  self.imu_bng   game engine: the AdvancedIMU (pose, acceleration) at 100 Hz sim time, polled in the imu thread
  one more       both cameras, polled in one burst in cameras.CameraStream (a fourth for --capture)
The game runs in real time; nothing here steps it. Closing disconnects and removes our sensors, and never quits the game.
"""
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import AdvancedIMU, Electrics

from openpilot.tools.sim.bridge.beamng import frame, scenario
from openpilot.tools.sim.bridge.beamng.cameras import MOUNT, CameraRig
from openpilot.tools.sim.bridge.beamng.pose import Pose, after_last_jump
from openpilot.tools.sim.bridge.beamng.tesla import GEAR_D, GEAR_N, GEAR_P, GEAR_R, GameLights
from openpilot.tools.sim.lib.common import SimulatorState, World, vec3

TESLA_STEER_RATIO = 12.0  # opendbc's Model 3; the game car is presented with this ratio, so paramsd needs no seeding
GEARS = {"P": GEAR_P, "R": GEAR_R, "N": GEAR_N, "D": GEAR_D}
BEAMNG_GEAR_D = 2  # vehicle.control(gear=) index of D in realistic_automatic


@dataclass
class BeamNGConfig:
  host: str = "192.168.137.1"
  port: int = 25252
  level: str = "west_coast_usa"
  scenario: str = "tp_bridge"
  # ETK 800: a 4.67 m long, 2.0 m wide (with mirrors), 1.39 m tall midsize sedan, against the Model 3's 4.69, 2.09 and 1.44 m
  model: str = "etk800"
  steer_lock_deg: float = 38.0  # etk800 road wheel angle at full steering input (510 deg at the wheel), from yaw rate on WCUSA
  # downtown west_coast_usa, eastbound in the right-hand lane, 140 m of straight two-way road before the signalized junction
  # group_80_81_82_83 (gate1's default approach). The (-717.1, 101, 118.7) pose from the PC agent sits in the left lane
  # facing oncoming traffic, 20 m from a junction.
  spawn_pos: tuple = (-908.82, 659.67, 112.25)
  spawn_rot: tuple = (0.0, 0.0, -0.7068525, 0.7073609)
  green: bool = True  # force every traffic signal green and hold it (nav runs don't test the model on lights)
  tod: float = 0.05  # BeamNG time of day, held (0 is noon): 13:12, high sun; the level's 08:52 start blinds an eastbound camera
  reload: bool = False  # reload the scenario even if ours is already loaded (a load takes about a minute)
  camera_update: float = 0.02  # s; below the game's frame time, so both cameras render on every frame
  camera_poll: float = 0.03  # s between polls of both cameras; the game answers once a frame, so polls beyond its fps wait
  lean: bool = True  # hide the HUD and particles: more game frames, so more camera frames
  capture: str = ""  # mp4 path for the chase view (capture.py)
  capture_fps: float = 10.0
  restore: bool = False  # on exit, undo the global game changes above (signals, HUD, particles, light): the game is shared
  # run_bridge.py --car-signals: write TurnPilotCarSignals on and a sim verified record before the car's first CAN, so the
  # car signals TurnPilot's turns itself (tesla.py car_signals; the bridge also takes it when the param is on or check)
  car_signals: bool = False


def load_scenario(bng, cfg: BeamNGConfig, log=print) -> tuple[Vehicle, float]:
  """Load the bridge's scenario, or reuse it when the game already has it, and set the game up as cfg says, with the
  ego at cfg's spawn. Returns the ego and the level's time of day from before (for restore_game). The bridge and the
  chase re-render (chase_render.py) share it, so a re-render sees the game the drive saw."""
  ego = None
  if not cfg.reload:
    try:
      sc = bng.scenario.get_current()
      v = sc.vehicles.get('ego')
      if sc.name == cfg.scenario and getattr(sc.level, 'name', sc.level) == cfg.level and v is not None and v.model == cfg.model:
        ego = v
        log(f"reusing the loaded scenario {cfg.scenario}")
    except Exception as e:
      log(f"no reusable scenario ({e!r})")
  if ego is None:
    sc = Scenario(cfg.level, cfg.scenario, description="TurnPilot sim bridge")
    ego = Vehicle('ego', model=cfg.model, license='TURNPLT')
    sc.add_vehicle(ego, pos=cfg.spawn_pos, rot_quat=cfg.spawn_rot)
    t0 = time.monotonic()
    sc.make(bng)
    bng.scenario.load(sc)
    bng.scenario.start()
    log(f"loaded {cfg.level} in {time.monotonic() - t0:.0f} s")
  bng.control.resume()
  tod0 = float(bng.env.get_tod()['time'])
  if cfg.lean:
    bng.ui.hide_hud()
    bng.settings.set_particles_enabled(False)
  bng.env.set_tod(tod=cfg.tod, play=False)
  if cfg.green and cfg.level == 'west_coast_usa':
    scenario.all_green(bng)
  bng.vehicles.teleport('ego', cfg.spawn_pos, cfg.spawn_rot)
  return ego, tod0


class BeamNGState(SimulatorState):
  def __init__(self):
    super().__init__()
    self.wheel_speed = 0.0
    self.gear = GEAR_D
    self.user_torque_nm = 0.0
    self.position = vec3(0, 0, 0)
    self.yaw_rate = 0.0  # rad/s, left positive
    self.a_long = 0.0  # m/s^2 along the car


class BeamNGWorld(World):
  def __init__(self, cfg: BeamNGConfig, log=print):
    super().__init__(dual_camera=True)
    self.cfg, self.log = cfg, log
    self.stop = threading.Event()
    self.bng_lock = threading.Lock()
    self.vlock = threading.Lock()
    self.threads: list[threading.Thread] = []
    self.ctrl = (0.0, 0.0, 1.0)  # BeamNG steering input (-1..1, right positive), throttle, brake
    self.game_lights = GameLights()  # the indicator, following the car's body (tesla.py car_signals)
    self.pose: Pose | None = None
    self.el: dict | None = None
    self.vehicle_rate = 0.0
    self.imu = self.rig = self.capture = None
    self.tod0 = scenario.LEVEL_TOD

    self.bng = BeamNGpy(cfg.host, cfg.port).open(launch=False)
    self.ego = self.setup_scenario()
    self.electrics = Electrics()
    self.imu_bng = BeamNGpy(cfg.host, cfg.port).open(launch=False)
    with self.bng_lock:
      self.bng.connection.send({"type": "CloseAdvancedIMU", "name": "tp_imu", "vid": "ego"}).recv()  # one left by a crashed run
      self.imu = AdvancedIMU("tp_imu", self.imu_bng, self.ego, gfx_update_time=0.0, physics_update_time=0.01, pos=MOUNT,
                             dir=(0, -1, 0), up=(0, 0, 1), smoother_strength=1.0, is_send_immediately=False, is_using_gravity=False,
                             is_visualised=False)
    self.rig = CameraRig(self.bng, self.bng_lock, self.ego, cfg.host, cfg.port, self.exit_event, cfg.camera_update, cfg.camera_poll,
                         log=log)
    if cfg.capture:
      from openpilot.tools.sim.bridge.beamng.capture import ChaseCapture
      self.capture = ChaseCapture(self.bng, self.bng_lock, self.ego, cfg.host, cfg.port, cfg.capture, log=log, fps=cfg.capture_fps)

    self.threads += [threading.Thread(target=self.vehicle_loop, name="bng_vehicle", daemon=True),
                     threading.Thread(target=self.imu_loop, name="bng_imu", daemon=True)]
    for t in self.threads:
      t.start()
    self.rig.start(self.stop)
    if self.capture:
      self.capture.start(self.stop)

  # *** setup ***
  def setup_scenario(self) -> Vehicle:
    with self.bng_lock:
      ego, self.tod0 = load_scenario(self.bng, self.cfg, self.log)
    ego.ai.set_mode('disabled')
    ego.set_shift_mode('realistic_automatic')
    ego.control(gear=BEAMNG_GEAR_D, steering=0.0, throttle=0.0, brake=1.0)
    return ego

  # *** threads ***
  def vehicle_loop(self):
    req = {"type": "SensorRequest", "sensors": {"electrics": self.electrics.encode_vehicle_request()}}
    n, t_rate = 0, time.monotonic()
    while not self.stop.is_set():
      t0 = time.monotonic()
      s, th, br = self.ctrl
      ctrl = {"type": "Control", "steering": s, "throttle": th, "brake": br}
      if self.el is not None and self.el.get('gear') != 'D':
        ctrl["gear"] = BEAMNG_GEAR_D  # a teleport resets the car to P: the Model 3 stays in D
      try:
        with self.vlock:
          conn = self.ego.connection
          rc = conn.send(ctrl)
          lights = self.game_lights.take()
          rl = conn.send(lights) if lights is not None else None
          re = conn.send(req)
          rc.ack("Controlled")
          if rl is not None:
            rl.ack("LightsSet")
          el = self.electrics.decode_response(re.recv("SensorData")["data"]["electrics"])
      except Exception as e:
        if self.stop.is_set():
          break
        self.game_lights.failed()
        self.log(f"vehicle link error: {e!r}")
        time.sleep(0.5)
        continue
      if el:
        self.el = el
      n += 1
      if t0 - t_rate > 5.0:
        self.vehicle_rate, n, t_rate = n / (t0 - t_rate), 0, t0
      time.sleep(max(0.0, 0.01 - (time.monotonic() - t0)))

  def imu_loop(self):
    history: deque = deque(maxlen=20)
    while not self.stop.is_set():
      t0 = time.monotonic()
      try:
        readings = self.imu.poll()
      except Exception as e:
        if self.stop.is_set():
          break
        self.log(f"imu error: {e!r}")
        time.sleep(0.5)
        continue
      if isinstance(readings, dict):
        readings = list(readings.values())
      latest = sorted((r for r in readings if 'dirX' in r and (not history or r['time'] > history[-1]['time'])), key=lambda r: r['time'])
      if latest:
        # a teleport, at the seam or inside the batch: don't difference across it, or the IMU reads a 30 rad/s spin
        seam = list(history)[-1:]
        k = after_last_jump(seam + latest)
        if k > 0:
          history.clear()
          latest = latest[k - len(seam):]
        history.extend(latest)
        self.pose = Pose(history, latest)
      time.sleep(max(0.0, 0.02 - (time.monotonic() - t0)))

  # *** World ***
  def apply_controls(self, steer_angle_deg, throttle_out, brake_out):
    """steer_angle_deg: steering wheel angle in openpilot's convention (left positive) at the Model 3's ratio."""
    road_wheel = steer_angle_deg / TESLA_STEER_RATIO
    s = float(np.clip(-road_wheel / self.cfg.steer_lock_deg, -1.0, 1.0))
    self.ctrl = (s, float(np.clip(throttle_out, 0, 1)), float(np.clip(brake_out, 0, 1)))

  def read_sensors(self, simulator_state: SimulatorState, /):
    state = simulator_state
    assert isinstance(state, BeamNGState)
    pose, el = self.pose, self.el
    if pose is None or el is None:
      return
    x, y, z = pose.pos
    state.position = vec3(float(x), float(y), float(z))
    state.velocity = vec3(*(float(v) for v in pose.vel))
    state.bearing = pose.bearing
    state.gps.latitude, state.gps.longitude, state.gps.altitude = frame.to_latlon(float(x), float(y), float(z))
    state.imu.accelerometer = vec3(*pose.accel)
    state.imu.gyroscope = vec3(*pose.gyro)
    state.imu.bearing = pose.bearing
    state.yaw_rate = pose.yaw_rate
    state.a_long = pose.a_long
    state.gear = GEARS.get(str(el.get('gear', 'D')), GEAR_D)
    speed = float(el.get('wheelspeed', 0.0))
    state.wheel_speed = -speed if state.gear == GEAR_R else speed
    state.steering_angle = -float(el.get('steering_input', 0.0)) * self.cfg.steer_lock_deg * TESLA_STEER_RATIO
    state.valid = True

  def tick(self):
    pass

  def read_state(self):
    pass

  def read_cameras(self):
    pass

  def reset(self):
    self.teleport(self.cfg.spawn_pos, self.cfg.spawn_rot)

  def teleport(self, pos, rot_quat):
    with self.bng_lock:
      self.bng.vehicles.teleport('ego', pos, rot_quat)
    with self.vlock:  # a teleport resets the car to P
      self.ego.control(gear=BEAMNG_GEAR_D)

  def close(self, reason: str = ""):
    if self.stop.is_set():
      return
    self.log(f"closing BeamNG world: {reason}")
    self.stop.set()
    for t in self.threads:
      t.join(timeout=2.0)
    for part in (self.capture, self.rig):
      if part is not None:
        part.close()
    try:
      with self.vlock:
        self.ego.control(steering=0.0, throttle=0.0, brake=1.0)
    except Exception as e:
      self.log(f"final brake failed: {e!r}")
    with self.bng_lock:
      try:
        self.imu.remove()
      except Exception as e:
        self.log(f"imu remove failed: {e!r}")
      if self.cfg.restore:
        try:
          scenario.restore_game(self.bng, self.tod0, scenario.load_signals() if self.cfg.level == 'west_coast_usa' else [])
          self.log("game restored: signals running, HUD and particles on, time of day back")
        except Exception as e:
          self.log(f"game restore failed: {e!r}")
    self.imu_bng.disconnect()
    self.bng.disconnect()  # leaves the game and the scenario running for the next start
    self.exit_event.set()
