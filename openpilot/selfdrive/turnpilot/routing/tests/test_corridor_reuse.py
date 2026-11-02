"""MapRouter.reuse: a corridor already on the comma (in memory, or saved under packs_dir) serves a later drive on the same
trip or its way back, with no download, after a reboot too. Against the fake map server over real TLS, Valhalla stubbed."""
import json
import os
from pathlib import Path

from openpilot.selfdrive.turnpilot.maps import corridor
from openpilot.selfdrive.turnpilot.maps.coverage import save
from openpilot.selfdrive.turnpilot.maps.tests.fakes import coverage_doc, level2_tile, make_pack
from openpilot.selfdrive.turnpilot.routing.sources import PACKS_DIR, MapRouter
from openpilot.selfdrive.turnpilot.routing.tests.test_sources import DESTINATION, START, STEPS, FakeValhalla, RouterCase

ELSEWHERE = (START[0] + 0.03, START[1] + 0.004)  # in the pack's tile, 2.6 km from the start and 1.7 km from the destination
FAR = (START[0] + 0.11, START[1] + 0.03)  # in the pack's tile too, 9.6 km from the start (START and DESTINATION are under 1 km apart)
ANOTHER = (START[0] + 0.07, START[1] + 0.004)  # in the pack's tile, far from all of those
OUTSIDE = (START[0] + 0.27, START[1] + 0.03)  # in the next tile east


class TestCorridorReuse(RouterCase):
  def rebooted(self) -> MapRouter:
    """A new router on the same comma storage, as navd after a reboot."""
    self.built.clear()
    return MapRouter(self.params, self.maps, valhalla=FakeValhalla(self))

  def first_drive(self):
    self.router.route(START, DESTINATION, 90, 'en')
    self.assertEqual(len(self.server.corridor_requests()), 1)

  def test_the_trip_is_saved_beside_the_manifest(self):
    self.first_drive()
    pack = self.router.corridor.pack
    self.assertEqual(json.loads((pack.path / corridor.TRIP).read_text()), {'start': list(START), 'destination': list(DESTINATION)})
    ((saved, start, destination),) = corridor.saved(self.maps / PACKS_DIR)
    self.assertEqual((saved.path, start, destination), (pack.path, START, DESTINATION))

  def test_after_a_reboot_the_same_destination_routes_on_the_saved_corridor(self):
    self.first_drive()
    pack = self.router.corridor.pack.path
    router = self.rebooted()
    self.params.writes.clear()
    nearby = (START[0] + 0.005, START[1] + 0.001)  # the car parked somewhere else on the corridor's roads
    self.assertEqual(router.route(nearby, DESTINATION, 0, 'en'), STEPS)
    self.assertEqual(len(self.server.corridor_requests()), 1)  # no download
    (source,) = self.built
    self.assertEqual((Path(source.tiles), source.calls), (pack, [(nearby, DESTINATION, 0, 'en')]))
    self.assertEqual(router.served[3], pack)  # the controls read the pack's own sidecar
    self.assertEqual(self.states(), ['checking', 'ready'])
    self.assertEqual(router.corridor.destination, DESTINATION)
    self.assertEqual(router.route(START, DESTINATION, 90, 'en'), STEPS)  # and reroutes stay on it
    self.assertEqual((len(self.built), len(self.server.corridor_requests())), (1, 1))

  def test_the_way_back_routes_on_the_corridor_in_memory_and_saved(self):
    self.router.route(START, FAR, 90, 'en')
    first = self.built[0]
    back = (FAR[0] + 0.003, FAR[1]), (START[0], START[1] + 0.003)  # parked 250 m off its destination, home 330 m off its start
    self.assertEqual(self.router.route(*back, 180, 'en'), STEPS)
    self.assertEqual(len(self.server.corridor_requests()), 1)
    self.assertEqual((len(self.built), first.calls[-1]), (1, (*back, 180, 'en')))  # the Valhalla loaded, no new one
    self.assertEqual(self.router.corridor.destination, back[1])
    router = self.rebooted()
    self.assertEqual(router.route(*back, 180, 'en'), STEPS)
    self.assertEqual((len(self.built), len(self.server.corridor_requests())), (1, 1))

  def test_another_trip_downloads_its_own(self):
    self.router.route(START, FAR, 90, 'en')
    router = self.rebooted()
    router.route(START, ELSEWHERE, 90, 'en')  # the same start, another destination
    router.route((FAR[0] + 0.003, FAR[1]), ANOTHER, 90, 'en')  # from its destination, but not back to its start
    self.assertEqual(len(self.server.corridor_requests()), 3)

  def test_a_corridor_from_another_map_is_not_used(self):
    self.first_drive()
    save(coverage_doc(version='2026-11-01'), self.maps)  # the server rebuilt its map since
    self.rebooted().route(START, DESTINATION, 90, 'en')
    self.assertEqual(len(self.server.corridor_requests()), 2)

  def test_a_route_that_leaves_the_saved_corridor_downloads_one_that_holds_it(self):
    self.first_drive()
    self.steps = [{**STEPS[0], 'geometry': {'type': 'LineString', 'coordinates': [list(START), list(OUTSIDE)]}}] + STEPS[1:]
    self.server.corridor = (200, make_pack({level2_tile(*START): b'tile', level2_tile(*OUTSIDE): b'more'}), 'application/x-tar')
    router = self.rebooted()
    self.assertEqual(router.route(START, DESTINATION, 90, 'en'), self.steps)
    self.assertEqual(len(self.server.corridor_requests()), 2)
    self.assertIsNone(router.failure)
    self.assertEqual(self.status()['state'], 'ready')

  def test_a_saved_corridor_that_fails_to_route_is_passed_over_quietly(self):
    """Not a failure the destination waits out, nor an error on the screen: the download goes ahead."""
    self.first_drive()
    router = self.rebooted()
    real = router.valhalla

    def first_one_broken(tiles):
      source = real(tiles)
      if len(self.built) == 1:
        def broken(*args, **kwargs):
          raise RuntimeError('tiles gone')
        source.route = broken
      return source

    router.valhalla = first_one_broken
    self.params.writes.clear()
    self.assertEqual(router.route(START, DESTINATION, 90, 'en'), STEPS)
    self.assertEqual(len(self.server.corridor_requests()), 2)
    self.assertIsNone(router.failure)
    self.assertNotIn('error', self.states())

  def test_packs_from_before_trips_were_saved_are_left_alone(self):
    self.first_drive()
    os.unlink(self.router.corridor.pack.path / corridor.TRIP)
    self.assertEqual(corridor.saved(self.maps / PACKS_DIR), [])
    self.rebooted().route(START, DESTINATION, 90, 'en')
    self.assertEqual(len(self.server.corridor_requests()), 2)
