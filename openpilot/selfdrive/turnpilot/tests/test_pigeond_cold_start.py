"""pigeond.cold_start_once: one ublox cold start per install, never raising."""
from openpilot.system.ubloxd import pigeond


class FakeParams:
  def __init__(self, on):
    self.v = {"TurnPilotGpsColdStart": on}

  def get_bool(self, k):
    return self.v.get(k, False)

  def put_bool(self, k, v):
    self.v[k] = v


class FakePigeon:
  def __init__(self, fail=False):
    self.sent, self.fail = [], fail

  def send(self, dat):
    if self.fail:
      raise OSError("tty gone")
    self.sent.append(dat)


def run(monkeypatch, params, pigeon):
  monkeypatch.setattr(pigeond, "Params", lambda: params)
  monkeypatch.setattr(pigeond.time, "sleep", lambda s: None)
  pigeond.cold_start_once(pigeon)


def test_cold_starts_once_then_never_again(monkeypatch):
  params, pigeon = FakeParams(True), FakePigeon()
  run(monkeypatch, params, pigeon)
  assert params.v["TurnPilotGpsColdStart"] is False
  assert pigeon.sent[0] == b"\xB5\x62\x09\x14\x04\x00\x01\x00\x00\x00\x22\xf0"  # flash backup cleared
  assert pigeon.sent[1] == pigeond.add_ubx_checksum(b"\xB5\x62\x06\x04\x04\x00\xFF\xFF\x02\x00")  # cold, GNSS only
  assert pigeon.sent[2] == pigeond.add_ubx_checksum(b"\xB5\x62\x06\x04\x04\x00\x00\x00\x09\x00")  # GNSS start
  again = FakePigeon()
  run(monkeypatch, params, again)
  assert again.sent == []


def test_off_sends_nothing(monkeypatch):
  pigeon = FakePigeon()
  run(monkeypatch, FakeParams(False), pigeon)
  assert pigeon.sent == []


def test_a_failing_tty_never_raises_and_still_clears_the_flag(monkeypatch):
  params = FakeParams(True)
  run(monkeypatch, params, FakePigeon(fail=True))
  assert params.v["TurnPilotGpsColdStart"] is False
