#!/usr/bin/env python3
"""Find S23's route pair offline: an origin and destination on the sim map whose fastest route crosses a two-way stop and
whose avoided route meets no stop or yield. No game, no PC: the road graph, the committed mapped controls, and route_audit
on the stop-sign sim tiles (tools/turnpilot/build_sim_tiles.sh, into any out_dir).

  python -m openpilot.tools.sim.bridge.beamng.scenarios.twoway_pair --tiles ~/turnpilot_data/valhalla/west_coast_usa_signals

A two-way stop here is a junction where some approaches have their own mapped stop sign and the rest have no control
(no signal anywhere). For each stop approach with a road straight across it, the origin is in the right lane up to
SPAWN_BACK_M back from the junction, the nearest point clear of the junctions and of every mapped control at SPEED_KPH
(catalog.spawn_conflicts) with only the junction's own sign between it and the junction; the destination DEST_PAST_M
straight across. Then route_audit's audit and navd's avoidance loop on the trip (certified as navd would, mode auto, the
side views on), and the pairs whose first route meets the stop and whose kept route is clean, shortest detour first.
"""
import argparse
import json
import math

import numpy as np

from openpilot.tools.sim.bridge.beamng.roadgraph import RoadGraph, wrap
from openpilot.tools.sim.bridge.beamng.scenarios import catalog as c
from openpilot.tools.sim.bridge.beamng.scenarios.timing import arc_lengths, point_at

SPAWN_BACK_M = 200.
SPAWN_MIN_M = 90.
DEST_PAST_M = 120.
SPEED_KPH = 30.
OWN_M = 80.  # an approach's own control: on its last OWN_M


def approach_kinds(graph: RoadGraph, j: str, controls) -> dict[str, str]:
  """{inbound neighbour: the kind of its own control into j ('stop', 'signal', 'yield', 'none')}."""
  out = {}
  for a in graph.adj[j]:
    if j not in graph.out[a]:
      continue
    toward = c.walk_straight(graph, j, a, OWN_M)[::-1]
    own = c.own_control(c.controls_along(toward, controls), float(arc_lengths(toward)[-1]))
    out[a] = own[1].kind if own else 'none'
  return out


def is_two_way(kinds: dict[str, str]) -> bool:
  v = list(kinds.values())
  return 'stop' in v and 'none' in v and all(k in ('stop', 'none') for k in v)


def candidates(graph: RoadGraph) -> list[dict]:
  controls = c.mapped_controls(graph)
  out = []
  for j in graph.junctions():
    kinds = approach_kinds(graph, j, controls)
    if not is_two_way(kinds):
      continue
    for a, kind in kinds.items():
      if kind != 'stop':
        continue
      app = c.walk_straight(graph, j, a, 500.)
      total = float(arc_lengths(app)[-1])
      h_in = wrap(graph.arm_heading(app) + math.pi)
      across = [o for o in graph.adj[j] if o != a and o in graph.out[j] and
                abs(math.degrees(wrap(c.arm_out_heading(graph, j, o) - h_in))) < c.STRAIGHT_TOLERANCE_DEG]
      if not across:
        continue
      far = c.walk_straight(graph, j, across[0], DEST_PAST_M + 30.)
      if float(arc_lengths(far)[-1]) < DEST_PAST_M:
        continue
      lanes, radius = graph.lanes_toward(j, a), graph.radius[a]
      for s in np.arange(min(SPAWN_BACK_M, total - 5.), SPAWN_MIN_M, -10.):
        spawn, heading = c.lane_pose(app, float(s), lanes, radius, 'right')
        if any(np.hypot(*(graph.pos[k][:2] - np.asarray(spawn[:2]))) < c.CLEAR_OF_JUNCTION_M for k in graph.junctions()):
          continue
        toward, s_spawn, _ = c.approach_arm(graph, spawn, heading, graph.pos[j][:2])
        along = c.controls_along(toward, controls)
        end = float(arc_lengths(toward)[-1])
        if c.spawn_conflicts(s_spawn, along, SPEED_KPH) or [x for sc, x in along if s_spawn < sc < end - OWN_M]:
          continue
        p, _ = point_at(far, DEST_PAST_M)
        out.append({'node': j, 'xy': [round(float(graph.pos[j][0]), 1), round(float(graph.pos[j][1]), 1)], 'approach': a,
                    'kinds': kinds, 'spawn_before_m': float(s), 'spawn': [round(float(v), 2) for v in spawn],
                    'heading_deg': round(math.degrees(heading), 1), 'dest': [round(float(v), 2) for v in p]})
        break
  return out


def audit(tiles: str, cands: list[dict]) -> list[dict]:
  """route_audit's audit and avoidance on each candidate's trip."""
  from openpilot.selfdrive.turnpilot.maps import surveys
  from openpilot.selfdrive.turnpilot.routing import certify, junctions
  from openpilot.selfdrive.turnpilot.routing.controls import route_controls
  from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource
  from openpilot.tools.sim.bridge.beamng import frame
  from openpilot.tools.turnpilot import route_audit as ra
  context = junctions.Context(surveys.load(surveys.SHIPPED).entries, {}, True, '')
  source = ValhallaSource(tiles)
  call, tiles_dir = source._call(), source.tiles_dir
  out = []
  for cnd in cands:
    olat, olon, _ = frame.to_latlon(*cnd['spawn'])
    dlat, dlon, _ = frame.to_latlon(*cnd['dest'])

    def route(exclude=None, o=(olon, olat), d=(dlon, dlat)):
      try:
        return source.route(o, d, None, None, exclude=exclude)
      except Exception:  # no route around: the loop keeps its best (as navd's RouteJob)
        return None
    steps = route()
    if not steps:
      continue
    report = ra.audit(call, steps, tiles_dir, tiles_dir, context, None, certify.Runtime(mode='auto', side_views=True), None)
    res = ra.audit_avoidance(report, steps, route, lambda s: route_controls(call, s, tiles_dir, tiles_dir, context=context),
                             headings=source.exclusion_headings)
    out.append({**cnd, 'origin': [olat, olon], 'destination': [dlat, dlon],
                'fastest': {'m': round(sum(float(s.get('distance', 0.)) for s in steps)), 's': round(sum(float(s.get('duration', 0.)) for s in steps)),
                            'stops': [[x.kind, round(x.route_m), x.certification] for x in report.controls if x.kind in ('stop', 'yield')]},
                'avoided': {'m': round(sum(float(s.get('distance', 0.)) for s in res.steps)),
                            'stops': [[x.kind, round(x.route_m), x.certification] for x in res.controls if x.kind in ('stop', 'yield')],
                            'signals_m': [round(x.route_m) for x in res.controls if x.kind == 'signal']},
                'avoidance': {'status': res.status, 'avoided': res.avoided, 'added_s': round(res.added_s, 1)}})
  return out


def good(r: dict) -> bool:
  return bool(r['fastest']['stops']) and not r['avoided']['stops'] and r['avoidance']['status'] == 'clean'


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--tiles', required=True, help="the stop-sign sim tiles (what TurnPilotValhallaTiles takes)")
  ap.add_argument('--json', action='store_true')
  args = ap.parse_args()
  rows = sorted(audit(args.tiles, candidates(RoadGraph())), key=lambda r: (not good(r), r['avoidance']['added_s']))
  if args.json:
    print(json.dumps(rows, indent=1))
    return
  for r in rows:
    print(f"{'GOOD' if good(r) else '    '} {r['node']} {r['xy']} from {r['approach']}, spawn {r['spawn']} heading {r['heading_deg']} " +
          f"to {r['dest']}: fastest {r['fastest']['m']} m, stops {r['fastest']['stops']}; avoided {r['avoided']['m']} m, " +
          f"stops {r['avoided']['stops']}, avoidance {r['avoidance']}")


if __name__ == '__main__':
  main()
