import time
from collections.abc import Callable
from multiprocessing import Queue

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.turnpilot.car_signals import read_mode
from openpilot.tools.sim.bridge.common import QueueMessageType, SimulatorBridge
from openpilot.tools.sim.bridge.beamng.control import AccelController
from openpilot.tools.sim.bridge.beamng.driver import PromptedDriver
from openpilot.tools.sim.bridge.beamng.sensors import BeamNGSensors
from openpilot.tools.sim.bridge.beamng.tesla import SimulatedTesla, write_sim_car_signals
from openpilot.tools.sim.bridge.beamng.world import BeamNGConfig, BeamNGState, BeamNGWorld

MANUAL_STEER_DEG = 90.0  # steering wheel degrees for a full manual steer input
STEER_NM_PER_UNIT = 12.5  # manual steer input as torsion bar torque; the port reads > 1 Nm as the driver steering
SET_SPEED_STEP_KPH = 5.0
DT = 0.01  # s, the loop period
ROLLING_START_ACCEL = 1.0  # m/s^2, the driver pulling away before engaging


class BeamNGBridge(SimulatorBridge):
  """openpilot on this Mac driving a car in BeamNG.tech, as a comma four in a Tesla Model 3.

  Queue commands (tools/sim/bridge/common.py control_cmd_gen, from the keyboard, the CLI or a scenario script):
  cruise_up engages (or raises the set speed by 5 km/h), cruise_down engages at the current speed (or lowers it),
  cruise_cancel cancels, plus upstream's steer_/throttle_/brake_/blinker_ commands, reset and quit; with the car's own
  turn signal (car_signals: TurnPilotCarSignals on or check at the start, or --car-signals) blinker_ works the driver's
  stalk in the car's body and hazard toggles its hazards.
  `scenario` runs in the bridge process with the world after the first sensor data: scenario(bridge) -> None, called at
  100 Hz until it returns True (done)."""

  def __init__(self, cfg: BeamNGConfig, engage: bool = False, set_speed_kph: float = 0.0, rolling_start_kph: float = 0.0,
               scenario: Callable[['BeamNGBridge'], bool | None] | None = None, test_run: bool = False):
    super().__init__(dual_camera=True, high_quality=False)
    self.cfg = cfg
    self.auto_engage = engage
    self.set_speed_kph = set_speed_kph
    self.rolling_start_kph = rolling_start_kph
    self.rolled = False
    self.scenario = scenario
    self.test_run = test_run
    self.engage_t = 0.0
    self.driver_brake = 0.0  # a scenario's driver on the brake pedal, 0..1 (which cancels cruise, as in the car)
    self.driver_gas = 0.0  # and on the gas, 0..1 (openpilot stays engaged: the gas overrides)

  def spawn_world(self, q: Queue):
    return BeamNGWorld(self.cfg)

  def handle_command(self, cmd: str, state: BeamNGState):
    m = cmd.split('_')
    car = self.car
    if m[0] == "cruise":
      if m[1] in ("up", "down") and car.engaged:
        car.adjust_set_speed(SET_SPEED_STEP_KPH if m[1] == "up" else -SET_SPEED_STEP_KPH)
      elif m[1] == "up":
        car.engage(state.wheel_speed, self.set_speed_kph)
      elif m[1] == "down":
        car.engage(state.wheel_speed)
      elif m[1] == "cancel":
        car.cancel()
    elif m[0] == "steer":
      self.manual[0] = float(m[1])
    elif m[0] == "throttle":
      self.manual[1] = float(m[1])
    elif m[0] == "brake":
      self.manual[2] = float(m[1])
    elif m[0] == "blinker" and car.body is not None:  # the driver's stalk in the body: a CANCEL clears it, as on the car
      if m[1] in ("left", "right"):
        car.toggle_stalk(m[1])
    elif m[0] == "blinker":  # the stalk latches: the same side again turns it off
      want = {"left": (True, False), "right": (False, True)}.get(m[1], (False, False))
      self.blinker = (False, False) if want == self.blinker else want
    elif m[0] == "hazard" and car.body is not None:
      car.toggle_hazards()
    elif m[0] == "ignition":
      state.ignition = not state.ignition
    elif m[0] == "reset":
      self.world.reset()
    elif m[0] == "quit":
      self.shutdown()

  def _run(self, q: Queue):
    self.world = self.spawn_world(q)
    # before the car's first CAN, so card's resolver reads it when it fingerprints the car
    params = Params()
    if self.cfg.car_signals:
      write_sim_car_signals(params)
    self.car = SimulatedTesla(car_signals=read_mode(params) in ('on', 'check'))
    self.sensors = BeamNGSensors()
    self.accel = AccelController()
    self.driver_accel = AccelController()
    state = self.simulator_state = BeamNGState()
    self.manual = [0.0, 0.0, 0.0]  # steer -1..1 (right positive, like upstream's keyboard), throttle, brake
    self.blinker = (False, False)
    self.driver = PromptedDriver()
    rk = Ratekeeper(100, print_delay_threshold=None)
    scenario_done = self.scenario is None
    last_print = 0.0
    t_prev = time.monotonic()

    while self._keep_alive:
      self.manual = [0.0, 0.0, 0.0]
      while not q.empty():
        msg = q.get()
        if getattr(msg, 'type', None) == QueueMessageType.CONTROL_COMMAND:
          self.handle_command(msg.info, state)

      now = time.monotonic()
      dt, t_prev = min(now - t_prev, 5 * DT), now
      self.world.read_sensors(state)
      steer_in, gas, brake = self.manual
      brake = max(brake, self.driver_brake)
      gas = max(gas, self.driver_gas)
      if self.auto_engage and state.valid:
        gas = max(gas, self.engage_step(state, dt))
      state.user_gas, state.user_brake = gas, brake
      state.user_torque_nm = -steer_in * STEER_NM_PER_UNIT
      nav_left, nav_right = self.driver.update()
      if self.car.body is not None:
        self.car.driver_hold((nav_left, nav_right))
        state.left_blinker, state.right_blinker = self.car.stalk
      else:
        state.left_blinker, state.right_blinker = self.blinker[0] or nav_left, self.blinker[1] or nav_right

      self.sensors.update(state, self.world)
      self.car.update(state)
      if self.car.body is not None:
        self.world.game_lights.set(self.car.lights)

      # what the Model 3's EPS and drive inverter would do with openpilot's DAS_steeringControl and DAS_control
      if self.car.lat_active() and steer_in == 0.0:
        steer = self.car.steer_cmd
      else:
        steer = -steer_in * MANUAL_STEER_DEG
      if self.car.long_active() and gas == 0.0 and brake == 0.0:
        throttle, brake_out = self.accel.update(self.car.accel_cmd, state.wheel_speed, dt, state.a_long)
      else:
        self.accel.reset()
        throttle, brake_out = gas, brake
        if gas == 0.0 and brake == 0.0 and not self.car.engaged and abs(state.wheel_speed) < 1.0:
          brake_out = 0.5  # nobody driving: hold the car instead of letting it creep in D
      self.world.apply_controls(steer, throttle, brake_out)
      state.is_engaged = self.car.engaged

      if self.scenario is not None and not scenario_done:
        self.rk_frame = rk.frame
        scenario_done = bool(self.scenario(self))

      if self.world.exit_event.is_set():
        self.shutdown()

      if not self.test_run and now - last_print > 1.0:
        last_print = now
        self.print_status()
      self.started.value = True
      rk.keep_time()

    if not scenario_done and hasattr(self.scenario, 'quit'):
      self.scenario.quit(self)  # a scenario cut short (quit, SIGTERM) still saves what it has and cleans up

  def engage_step(self, state: BeamNGState, dt: float) -> float:
    """Engage like a driver: with a rolling start, press the gas up to that speed first (the driving model holds a car
    that stands on an empty road: its standstill accel stays under drive_helpers.should_stop's 0.1), then pull the stalk
    when openpilot is engageable; if openpilot didn't take the rising edge, cancel and retry. Returns the driver's gas."""
    sm = self.car.sm
    sm.update(0)
    now = time.monotonic()
    roll_to = self.rolling_start_kph / 3.6 - 0.5
    if sm['selfdriveState'].enabled:
      self.auto_engage = self.rolled = False
    elif not self.car.engaged and not self.rolled and state.wheel_speed < roll_to:
      throttle, _ = self.driver_accel.update(ROLLING_START_ACCEL, state.wheel_speed, dt)
      return max(throttle, 0.05)
    else:
      # off the gas once rolling, since a pressed pedal blocks engagement; roll again if the car slowed right down
      self.rolled = state.wheel_speed > roll_to - 2.0
      if not self.car.engaged and sm['selfdriveState'].engageable and now - self.engage_t > 1.0:
        self.car.engage(state.wheel_speed, self.set_speed_kph)
        self.engage_t = now
      elif self.car.engaged and now - self.engage_t > 2.0:
        self.car.cancel()
        self.engage_t = now
    self.driver_accel.reset()
    return 0.0

  def print_status(self):
    s = self.simulator_state
    c = self.car
    print(f"v {s.wheel_speed:5.2f} m/s  angle {s.steering_angle:6.1f}  cruise {'ON ' if c.engaged else 'off'} set {c.set_speed_kph:5.1f} km/h  " +
          f"lat {'on ' if c.lat_active() else 'off'} cmd {c.steer_cmd:6.1f}  long {'on ' if c.long_active() else 'off'} " +
          f"accel {c.accel_cmd:5.2f} (meas {s.a_long:5.2f})  vehicle link {self.world.vehicle_rate:4.1f} Hz", flush=True)

