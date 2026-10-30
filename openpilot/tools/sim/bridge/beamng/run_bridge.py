#!/usr/bin/env python3
"""BeamNG.tech bridge: openpilot on this Mac drives a car in stock BeamNG.tech on the Windows PC, as a comma four in a
Tesla Model 3. Start openpilot with ./launch_openpilot.sh next to this file, then this. Kept apart from
tools/sim/run_bridge.py, which imports metadrive unconditionally."""
import argparse
import signal
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from multiprocessing import Queue

from openpilot.tools.sim.bridge.common import control_cmd_gen
from openpilot.tools.sim.bridge.beamng.bridge import BeamNGBridge
from openpilot.tools.sim.bridge.beamng.world import BeamNGConfig


def floats(s: str) -> tuple:
  return tuple(float(x) for x in s.split(','))


def parse_args(add_args=None):
  d = BeamNGConfig()
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('--host', default=d.host)
  p.add_argument('--port', type=int, default=d.port)
  p.add_argument('--level', default=d.level)
  p.add_argument('--model', default=d.model, help='BeamNG vehicle model (steer lock is measured for etk800 only)')
  p.add_argument('--spawn', type=floats, default=d.spawn_pos, help='x,y,z in BeamNG world meters')
  p.add_argument('--rot', type=floats, default=d.spawn_rot, help='rotation quaternion qx,qy,qz,qw')
  p.add_argument('--reload', action='store_true', help='reload the scenario even if ours is loaded')
  p.add_argument('--engage', action='store_true', help='engage openpilot as soon as it is engageable')
  p.add_argument('--set-speed', type=float, default=0.0, help='cruise set speed on engage, km/h (default: current speed)')
  p.add_argument('--rolling-start', type=float, default=0.0, help='with --engage, drive up to this speed (km/h) before engaging')
  p.add_argument('--camera-update', type=float, default=d.camera_update, help='BeamNG camera requested update time, s')
  p.add_argument('--camera-poll', type=float, default=d.camera_poll, help='period between polls of both cameras, s')
  p.add_argument('--no-lean', action='store_true', help='keep the HUD and particles on')
  p.add_argument('--signals', action='store_true', help="leave the traffic signals running (default: all green and held)")
  p.add_argument('--capture', default='', help='record the chase view to this mp4')
  p.add_argument('--capture-fps', type=float, default=d.capture_fps, help='chase view frame rate (each render costs camera pairs)')
  p.add_argument('--restore-on-exit', action='store_true',
                 help='on exit, set the shared game back: signals running, HUD and particles on, the light as it was')
  p.add_argument('--duration', type=float, default=0.0, help='quit after this many seconds (0: run until q or Ctrl-C)')
  p.add_argument('--no-keyboard', action='store_true')
  p.add_argument('--light-oracle', action='store_true',
                 help="publish the game's light states ahead as turnpilotLights (light_oracle.py; SIM ONLY: turnpilotd takes them " +
                      "only with TURNPILOT_LIGHT_ORACLE=1 in launch_openpilot.sh)")
  p.add_argument('--light-latency', type=float, default=0.3, help='with --light-oracle, publish the lights this late, s')
  return p.parse_args(add_args)


def start_light_oracle(args):
  """light_oracle.py in its own process, on its own BeamNGpy connection (never the bridge's world lock)."""
  return subprocess.Popen([sys.executable, '-m', 'openpilot.tools.sim.bridge.beamng.light_oracle', '--host', args.host,
                           '--port', str(args.port), '--light-latency', str(args.light_latency)])


@contextmanager
def light_oracle(args):
  """The light oracle for the drive with --light-oracle (run_bridge.py, scenarios/run.py and route_drive.py all take the
  flag), stopped when it ends; None without the flag."""
  oracle = start_light_oracle(args) if args.light_oracle else None
  try:
    yield oracle
  finally:
    if oracle is not None and oracle.poll() is not None:  # it ended during the drive: light_oracle.EXIT_NOT_READY, a crash
      print(f"warning: the light oracle exited ({oracle.returncode}) during the drive: every light unknown since; its log says why",
            flush=True)
    if oracle is not None:
      oracle.terminate()
      try:
        oracle.wait(timeout=10)
      except subprocess.TimeoutExpired:  # still up: killed, never a finished drive turned into a failed one
        print("warning: the light oracle didn't stop on SIGTERM, killed", flush=True)
        oracle.kill()


def config_from_args(args) -> BeamNGConfig:
  return BeamNGConfig(host=args.host, port=args.port, level=args.level, model=args.model, spawn_pos=args.spawn, spawn_rot=args.rot,
                      reload=args.reload, camera_update=args.camera_update, camera_poll=args.camera_poll, lean=not args.no_lean,
                      green=not args.signals, capture=args.capture, capture_fps=args.capture_fps,
                      restore=args.restore_on_exit)


def main():
  args = parse_args()
  with light_oracle(args):
    drive(args)


def drive(args):
  q: Queue = Queue()
  bridge = BeamNGBridge(config_from_args(args), engage=args.engage, set_speed_kph=args.set_speed, rolling_start_kph=args.rolling_start)
  p = bridge.run(q)

  def stop(*_):
    q.put(control_cmd_gen("quit"))
  signal.signal(signal.SIGINT, stop)
  signal.signal(signal.SIGTERM, stop)

  if args.duration:
    t_end = time.monotonic() + args.duration
    while p.is_alive() and time.monotonic() < t_end:
      time.sleep(0.2)
    stop()
  elif args.no_keyboard:
    pass
  elif sys.stdin.isatty():
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(q)
  else:
    # scripted: one command per line on stdin, e.g. cruise_up, cruise_cancel, blinker_left, quit; at the end of stdin
    # (a closed pipe, < /dev/null, nohup) the bridge keeps running as with --no-keyboard, until quit, Ctrl-C or SIGTERM
    def read_commands():
      for line in sys.stdin:
        if line.strip():
          q.put(control_cmd_gen(line.strip()))
    threading.Thread(target=read_commands, daemon=True).start()
  p.join()


if __name__ == "__main__":
  main()
