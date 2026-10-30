#!/usr/bin/env python3
"""Chase view of a drive, rendered after it from the drive's pose log: the car is teleported to each logged pose and one
frame is taken from capture.py's chase camera, at 20 Hz, into an mp4 with the `<mp4>.start.json` sidecar demo_video.py
aligns on.

Why after and not during: a third camera costs the driving cameras 6 to 9 pairs a second whatever its rate, and turns
fail at 11 to 14 (README, Routed drives). BeamNG's own replay (core_replay) was tried on 2026-10-27: recording held the
bridge's 20 pairs a second with the car standing (not measured while driving), but its clock ran at about a third of
wall time and camera sensors render nothing while it is paused, so a replay can't be lined up with the UI or captured
frame by frame. Teleporting needs only the log, so it also covers drives recorded before this existed.

The poses come from the drive's IMU pose log, cleaned and moved onto the game's clock by chase_track.py. Scenario
actors logged with the run (`actor_poses`) are put back too: the mannequin as a prop, cars as etk800s. So are the
lights and the ego's indicator the run logged (chase_track.Replay): a forced light's states from its light_events, the
light oracle's readings of the others, carState's blinkers. A light nothing logged shows what the render's world shows
(every light green and held, as the bridge without --signals): the sidecar says which were replayed, and whether the
run's own lights were running (signals_running), so a lamp the log can't vouch for is captioned as not the run's.

Run it with the game free: after the drive, with the bridge stopped (it teleports the ego the bridge drives).

  python -m openpilot.tools.sim.bridge.beamng.chase_render demo.json --run 1 -o demo_chase.mp4
"""
import argparse
import json
import os
import shutil
import subprocess
import time

import numpy as np
from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Camera

from openpilot.tools.sim.bridge.beamng import scenario
from openpilot.tools.sim.bridge.beamng.cameras import bng_recv, bng_send, fingerprint
from openpilot.tools.sim.bridge.beamng.capture import CH, CHASE_DIR, CHASE_POS, CW
from openpilot.tools.sim.bridge.beamng.chase_track import POSE_LAG_S, Replay, actor_at, ego_at, ego_track, lamp_timeline, light_timeline, teleport_pos
from openpilot.tools.sim.bridge.beamng.scenarios.director import LIGHT_STATES
from openpilot.tools.sim.bridge.beamng.world import BeamNGConfig, load_scenario

FPS = 20
FRESH_FRAMES = 4   # after a teleport the first two new frames can still show the old pose; the third is right, one spare
FRAME_TIMEOUT_S = 2.0
PROGRESS_EVERY = 100
CAMERA = "tp_chase_rr"
CAR_PREFIX = "tp_rr_"


def load_run(path: str, run: int) -> tuple[dict, dict]:
  with open(path) as f:
    doc = json.load(f)
  r = doc['runs'][run - 1]
  if r.get('pose_fields', [])[:9] != ["wall_s", "t_s", "x", "y", "z", "qx", "qy", "qz", "qw"] or len(r.get('poses', [])) < 2:
    raise ValueError(f"run {run} of {path} has no pose log (route_drive and the scenario runner write `poses`)")
  return doc, r


class Renderer:
  """The chase camera on the ego, read on its own connection the way cameras.CameraStream does."""

  def __init__(self, bng, ego, host: str, port: int, size=(CW, CH), log=print):
    self.bng, self.ego, self.log = bng, ego, log
    self.w, self.h = size
    bng.connection.send({"type": "CloseCamera", "name": CAMERA}).recv()  # one left by a killed render
    self.cam = Camera(CAMERA, bng, ego, requested_update_time=0.02, pos=CHASE_POS, dir=CHASE_DIR, up=(0, 0, 1),
                      resolution=size, field_of_view_y=55, near_far_planes=(0.1, 1000.0), is_using_shared_memory=False,
                      is_render_annotations=False, is_render_depth=False, is_render_instance=False)
    self.poller = BeamNGpy(host, port).open(launch=False)
    self.fp = None
    self.polls = 0
    self.cars: dict[str, Vehicle] = {}
    self.walkers: set[str] = set()

  def poll(self):
    conn = self.poller.connection
    buf = bng_recv(conn, bng_send(conn, type="PollCamera", name=CAMERA, isUsingSharedMemory=False))["data"].get("colour")  # codespell:ignore colour
    self.polls += 1
    if isinstance(buf, bytes) and len(buf) in (self.w * self.h * 3, self.w * self.h * 4):
      return buf
    return None

  def fresh_frame(self) -> np.ndarray | None:
    """The FRESH_FRAMES-th new render from now."""
    got, t_end, buf = 0, time.monotonic() + FRAME_TIMEOUT_S, None
    while got < FRESH_FRAMES and time.monotonic() < t_end:
      b = self.poll()
      if b is not None and fingerprint(b) != self.fp:
        self.fp, buf, got = fingerprint(b), b, got + 1
      else:
        time.sleep(0.005)
    if got < FRESH_FRAMES:
      return None
    return np.frombuffer(buf, dtype=np.uint8).reshape(self.h, self.w, -1)[:, :, :3]

  def place_ego(self, pose: np.ndarray):
    # no vehicle reset: it costs the game ~25 ms a frame, and the braked car keeps the pose without it (within 3 mm)
    self.bng.vehicles.teleport('ego', tuple(float(v) for v in teleport_pos(pose[:3], pose[3:])), tuple(float(v) for v in pose[3:]),
                               reset=False)

  def place_actor(self, name: str, kind: str, pose: tuple | None):
    """Put an actor where the log has it, spawning it the first time; outside its logged span it isn't in the game."""
    if pose is None:
      if name in self.walkers:
        scenario.remove_pedestrian(self.bng, CAR_PREFIX + name)
        self.walkers.discard(name)
      if name in self.cars:
        self.bng.vehicles.despawn(self.cars.pop(name))
      return
    x, y, z, h = pose
    if kind == 'walker':
      if name not in self.walkers:
        scenario.spawn_pedestrian(self.bng, CAR_PREFIX + name, x, y, z, h)
        self.walkers.add(name)
      else:
        scenario.move_pedestrian(self.bng, CAR_PREFIX + name, x, y, z, h)
      return
    if name not in self.cars:
      v = Vehicle(CAR_PREFIX + name, model='etk800', license='SCENARIO', color=(0.75, 0.75, 0.78))
      if not self.bng.vehicles.spawn(v, (x, y, z), scenario.heading_quat(h)):
        raise RuntimeError(f"couldn't spawn {name}")
      v.control(throttle=0.0, brake=1.0)
      self.cars[name] = v
    else:
      self.bng.vehicles.teleport(CAR_PREFIX + name, (x, y, z), scenario.heading_quat(h))

  def close(self):
    for name in self.walkers:
      scenario.remove_pedestrian(self.bng, CAR_PREFIX + name)
    for name, v in self.cars.items():
      try:
        self.bng.vehicles.despawn(v)
      except Exception as e:
        self.log(f"despawn {name} failed: {e!r}")
    try:
      self.cam.remove()
    except Exception as e:
      self.log(f"chase camera remove failed: {e!r}")
    self.poller.disconnect()


def write_sidecar(video: str, start: float, frames: int, lag: float, source: str, replayed: dict | None = None):
  with open(f"{video}.start.json", 'w') as f:
    json.dump({'start': start, 'end': start + (frames - 1) / FPS, 'frames': frames, 'fps': FPS, 'source': source,
               'pose_lag_s': lag, **(replayed or {})}, f)


def replayed_note(doc: dict, replay: Replay) -> dict:
  """What the sidecar says of the lights and indicator: which instances were replayed from the log, whether the run's
  own lights ran (--signals), and so whether any other lamp in view is the run's."""
  running = doc.get('signals_running')
  note = ("every light was green and held in the run, as here" if running is False else
          "the run's lights were running: only the replayed ones are the run's, any other lamp in view is not" if running else
          "the log doesn't say whether the run's lights were running: only the replayed ones are the run's")
  return {'lights_replayed': replay.instances(), 'indicator_replayed': bool(replay.lamps), 'signals_running': running,
          'lights_note': note}


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('log', help="route_drive's --out JSON or the scenario runner's result.json")
  ap.add_argument('--run', type=int, default=1)
  ap.add_argument('-o', '--out', required=True, help='mp4')
  ap.add_argument('--lag', type=float, default=POSE_LAG_S, help='s the logged pose trails the game')
  ap.add_argument('--from-s', type=float, default=0.0, help='start this many s into the log')
  ap.add_argument('--seconds', type=float, default=0.0, help='render only this long (0: to the end)')
  ap.add_argument('--host', default=BeamNGConfig.host)
  ap.add_argument('--port', type=int, default=BeamNGConfig.port)
  args = ap.parse_args(argv)

  doc, r = load_run(args.log, args.run)
  segs, t0, t1 = ego_track(r['poses'], args.lag)
  t0 += args.from_s
  if args.seconds > 0:
    t1 = min(t1, t0 + args.seconds)
  grid = np.arange(t0, t1, 1.0 / FPS)
  actors = r.get('actor_poses', {})
  print(f"{len(r['poses'])} poses, {sum(len(s) for s in segs)} kept in {len(segs)} segment(s), {len(grid)} frames " +
        f"({len(grid) / FPS:.0f} s), {len(actors)} actor(s)", flush=True)

  ffmpeg = shutil.which("ffmpeg")
  if ffmpeg is None:
    raise RuntimeError("chase_render needs ffmpeg on PATH")
  os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
  bng = BeamNGpy(args.host, args.port).open(launch=False)
  renderer, proc, frames, misses, replay = None, None, 0, 0, None
  try:
    ego, _ = load_scenario(bng, BeamNGConfig(host=args.host, port=args.port))
    ego.connect(bng)
    ego.control(throttle=0.0, brake=1.0, steering=0.0)
    renderer = Renderer(bng, ego, args.host, args.port)
    replay = Replay(light_timeline(r), lamp_timeline(r.get('rows', [])),
                    set_light=lambda inst, st: bng.traffic_signals.set_instance_strict_state(inst, LIGHT_STATES[st]),
                    set_lamp=lambda side: ego.set_lights(left_signal=side == 'left', right_signal=side == 'right'))
    if replay.lights:
      scenario.hold_signals(bng)  # a sequence would overwrite the replayed states
    print(f"replaying {len(replay.lights)} light(s) {replay.instances()} and {'the' if replay.lamps else 'no'} indicator", flush=True)
    proc = subprocess.Popen([ffmpeg, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{CW}x{CH}",
                             "-r", str(FPS), "-i", "pipe:0", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
                             "-pix_fmt", "yuv420p", args.out], stdin=subprocess.PIPE)
    t_block, last = time.monotonic(), None
    for k, t in enumerate(grid):
      renderer.place_ego(ego_at(segs, float(t)))
      for name, a in actors.items():
        renderer.place_actor(name, a['kind'], actor_at(a['rows'], float(t)))
      replay.apply(float(t))
      img = renderer.fresh_frame()
      if img is None:
        misses += 1
        img = last  # a frame the game never answered: repeat the last, so the video keeps its clock
      if img is None:
        continue
      proc.stdin.write(np.ascontiguousarray(img).tobytes())
      last, frames = img, frames + 1
      if (k + 1) % PROGRESS_EVERY == 0:  # a render that slows down shows where: the game (polls) or this Mac (load)
        el = time.monotonic() - t_block
        print(f"frame {k + 1}/{len(grid)}: {el / PROGRESS_EVERY * 1e3:.0f} ms a frame, {renderer.polls / PROGRESS_EVERY:.1f} polls a " +
              f"frame, Mac load {os.getloadavg()[0]:.1f}", flush=True)
        t_block, renderer.polls = time.monotonic(), 0
  finally:
    if proc is not None:
      proc.stdin.close()
      proc.wait(timeout=60)
    if replay is not None:
      try:
        replay.close(green=True)  # load_scenario left every light green and held: so again
      except Exception as e:
        print(f"warning: putting the replayed lights back failed: {e!r}", flush=True)
    if renderer is not None:
      renderer.close()
    bng.disconnect()
  if frames:
    write_sidecar(args.out, float(grid[len(grid) - frames]), frames, args.lag, f"chase re-render of {os.path.basename(args.log)} run {args.run}",
                  replayed_note(doc, replay) if replay is not None else None)
  print(f"{args.out}: {frames} frames ({frames / FPS:.1f} s) from wall time {grid[0]:.2f}, {misses} repeated after no answer", flush=True)


if __name__ == "__main__":
  main()
