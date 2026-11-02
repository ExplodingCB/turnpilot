"""The diagnostics report Chase copies from the phone page for Kellen (diagnostics.py): what it says, and that it never says
the map key, the web token or a position."""
import json
import os
import subprocess
import sys
import textwrap
from unittest import mock

from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot import diagnostics, preflight
from openpilot.selfdrive.turnpilot.diagnostics import (BIG_WHY_MAX, EVENTS_KEPT, EVENTS_PARAM, REPEAT_S, EventLog, Preflight, chestnut_line,
                                                        log_event, model_text, note_big_model, note_big_model_event, report)
from openpilot.selfdrive.turnpilot.web.store import last_gps_position
from openpilot.system.qcomgpsd import qcomgpsd

KEY = "tpk_live_7f3a9c2e5b1d8046"
SHA = "691321d4c0ffee5e1f2a3b4c5d6e7f8091a2b3c4"
FIX = "39.768403, -86.158102"  # Monument Circle


class BlockingParams(Params):
  def put(self, key, dat, block=True):
    super().put(key, dat, block=True)


class Clock:
  def __init__(self, t=1_761_940_000.):
    self.t = t

  def __call__(self):
    return self.t


def record(msg, level=40, daemon="turnpilotd", exc=None):
  rec = {"msg": msg, "ctx": {"daemon": daemon, "dongle_id": "0123456789abcdef"}, "levelnum": level, "created": 1.}
  if exc:
    rec["exc_info"] = exc
  return json.dumps(rec)


def plan(prompt="none", alert="none", missed=False, reason=""):
  return {"prompt": prompt, "alert": alert, "missed": missed, "reason": reason}


class TestReport(OpenpilotTestCase):
  def params(self):
    params = BlockingParams()
    params.put("GitCommit", SHA)
    params.put("GitBranch", "turnpilot")
    params.put("TurnPilotMapKey", KEY)
    params.put("TurnPilotWebToken", "tok-a1b2c3d4e5f6")
    return params

  def test_it_says_the_build_agnos_model_preflight_and_events(self):
    params = self.params()
    live = {"model_big": True, "native": "full", "far": "read", "model_s": 0.0213, "full_s": 0.0061, "far_s": 0.0120, "samples": 20}
    events = [{"t": 1_761_940_000., "what": "handback stalledTakeControl", "why": "Stalled in the turn: take control"},
              {"t": 1_761_940_090., "what": "error turnpilotd", "why": "turnpilotd failed (KeyError: 'x')", "n": 3}]
    lines = ["PASS  car                tesla TESLA_MODEL_3", "PASS  big driving model  big_driving_tinygrad.pkl and both warps are real files",
             "PASS  native detect      full: the whole frame natively", "PASS  chestnut           present, USB 5000 Mbps"]
    text = report(params, live, events, lines, "12.8")
    assert text.startswith("TurnPilot diagnostics, ")
    assert f"build: {SHA[:10]} on turnpilot" in text and "AGNOS: 12.8" in text
    assert "driving model: the big model on the chestnut" in text and "native detect: full (the whole frame natively)" in text
    assert "model 21.3 ms, full pass 6.1 ms, far slot 12.0 ms" in text
    assert "TESLA_MODEL_3" in text and "PASS  chestnut" in text
    newest, older = text.index("error turnpilotd (x3)"), text.index("handback stalledTakeControl: Stalled in the turn")
    assert newest < older  # newest first
    off = report(params, None, [], [], None)
    assert "driving model: not running" in off and "AGNOS: unknown" in off and "none since the last install" in off

  def test_it_never_says_the_key_the_token_or_a_position(self):
    params = self.params()
    token = params.get("TurnPilotWebToken")
    pre = preflight.render([preflight.check_car(None), preflight.check_token(token),
                            preflight.check_pairing_url(token, "192.168.43.2", 8088),
                            preflight.Check("map server", preflight.FAIL, f"refused key {KEY} at {FIX}", "check the key")])
    events = [{"t": 1., "what": "alert hardBrake", "why": f"braking for a person at {FIX}, key {KEY}"}]
    text = report(params, None, events, diagnostics.keep_lines(pre.splitlines()), "12.8")
    for secret in (KEY, token, "39.7684", "-86.1581", "8088/?t=", "pairing URL", "web token"):
      assert secret not in text, secret
    assert "[redacted]" in text and "[position]" in text and "FAIL  car" in text and "fix: check the key" in text


class TestEvents(OpenpilotTestCase):
  def test_onsets_are_kept_and_saved(self):
    params, clock = BlockingParams(), Clock()
    log = EventLog(params, clock)
    log.see_plan(plan())
    log.see_plan(plan("waitingForGap", "carStoppedNote"))  # visual and notes: not events
    assert log.get() == []
    for _ in range(3):  # held while it lasts: one event
      log.see_plan(plan("stalledTakeControl", reason="Stalled in the turn"))
    clock.t += 1
    log.see_plan(plan("stalledTakeControl", "hardBrake", reason="Person in path"))
    log.see_plan(plan(missed=True, reason="Turn missed"))
    assert [(e["what"], e["why"]) for e in log.get()] == [("handback stalledTakeControl", "Stalled in the turn"),
                                                        ("alert hardBrake", "Person in path"), ("turn missed", "Turn missed")]
    assert params.get(EVENTS_PARAM) == log.get() and EventLog(params).get() == log.get()  # kept across a reboot

  def test_repeats_count_and_the_oldest_go(self):
    params, clock = BlockingParams(), Clock()
    log = EventLog(params, clock)
    for _ in range(3):
      log.see_log(record("turnpilot detector failed", daemon="modeld", exc="Traceback\n  ...\nRuntimeError: chestnut failed with 7"))
      clock.t += 1
    assert log.get() == [{"t": clock.t - 1, "what": "error modeld", "why": "turnpilot detector failed (RuntimeError: chestnut failed with 7)", "n": 3}]
    clock.t += REPEAT_S
    for k in range(EVENTS_KEPT + 2):
      log.see_plan(None)  # turnpilotd stopped and started again: the next handback is an onset
      log.see_plan(plan("cantStartTakeControl", reason=f"try {k} at {FIX}"))
    kept = log.get()
    assert len(kept) == EVENTS_KEPT and kept[-1]["why"] == f"try {EVENTS_KEPT + 1} at [position], [position]"
    assert FIX.split(",", maxsplit=1)[0] not in json.dumps(params.get(EVENTS_PARAM))  # no position in the saved param

  def test_only_turnpilots_logged_errors(self):
    assert log_event(record("turnpilotd: lane paint off", level=30)) is None  # a warning
    assert log_event(record("thermal zone read failed", daemon="hardwared")) is None
    assert log_event("not json") is None
    assert log_event(record("plan failed", exc="Traceback\nKeyError: 'lead'")) == ("error turnpilotd", "plan failed (KeyError: 'lead')")
    assert log_event(record("turnpilot native schedule failed", daemon="modeld")) == ("error modeld", "turnpilot native schedule failed")
    assert log_event(record({"event": "turnpilot.hard_brake"}, daemon="turnpilot_web"))[0] == "error turnpilot_web"


class TestPreflight(OpenpilotTestCase):
  def test_run_at_most_every_cache_period_without_the_token_lines(self):
    token, runs, now = "tok-a1b2c3d4e5f6", [], [100.]

    def run():
      runs.append(1)
      return preflight.render([preflight.check_shadow(True, False), preflight.check_token(None),
                               preflight.check_pairing_url(token, "127.0.0.1", 8088), preflight.check_web(True, True)]).splitlines()
    pf = Preflight(run)
    lines = pf.get(lambda: now[0])
    assert any(ln.startswith("INFO  shadow mode") for ln in lines) and any(ln.startswith("PASS  web server") for ln in lines)
    assert not any("web token" in ln or "pairing URL" in ln or "TurnPilotWebToken" in ln or "fix: turnpilot_web" in ln for ln in lines)
    now[0] += diagnostics.PREFLIGHT_CACHE_S - 1
    assert pf.get(lambda: now[0]) == lines and len(runs) == 1
    now[0] += 1
    pf.get(lambda: now[0])
    assert len(runs) == 2

  def test_settings_line(self):
    params = BlockingParams()
    assert diagnostics.settings_line(params, None).startswith("build unknown, AGNOS unknown.")
    params.put("GitCommit", SHA)
    params.put("GitBranch", "turnpilot")
    line = diagnostics.settings_line(params, "12.8")
    assert line.startswith(f"build {SHA[:10]} on turnpilot, AGNOS 12.8.") and "phone page" in line


def raised(exc):
  try:
    raise exc
  except Exception as e:
    return e


class TestBigModel(OpenpilotTestCase):
  """modeld's big model failures (note_big_model): saved in TurnPilotDiagEvents, read back by the web app's EventLog and
  said on the driving model line."""

  def test_a_load_failure_says_its_exception_timings_try_and_wait(self):
    params, clock = BlockingParams(), Clock()
    params.put("TurnPilotMapKey", KEY)
    exc = raised(RuntimeError(f"chestnut failed with 7, key {KEY} at {FIX}"))
    note_big_model(params, "failed to load", exc, 12.34, False, 0.42, None, now=clock)
    [event] = params.get(EVENTS_PARAM)
    assert event["what"] == "big model failed to load" and event["t"] == clock.t
    why = event["why"]
    assert why.startswith("RuntimeError: chestnut failed with 7") and "raised after 12.3 s" in why and "the first try" in why
    assert "chestnut enumerated in 0.4 s" in why and "openpilot/selfdrive/turnpilot/tests/test_diagnostics.py:" in why and " in raised" in why
    assert KEY not in why and "39.7684" not in why and "/Users/" not in why and len(why) <= BIG_WHY_MAX

  def test_a_timeout_and_a_wait_that_raised_on_the_retry(self):
    params = BlockingParams()
    note_big_model(params, "timed out", None, 60, True, None, None)
    note_big_model(params, "failed to load", raised(TimeoutError("chestnut did not enumerate")), 10.05, True, None, "TimeoutError")
    timeout, waited = params.get(EVENTS_PARAM)
    assert timeout["what"] == "big model timed out" and timeout["why"].startswith("timed out after 60.0 s; the ChestnutRetry try")
    assert "wait_for_chestnut didn't return" in timeout["why"]
    expected = "TimeoutError: chestnut did not enumerate; raised after 10.1 s; the ChestnutRetry try; wait_for_chestnut raised"
    assert waited["why"].startswith(f"{expected} TimeoutError")

  def test_writing_never_raises(self):
    class Broken:
      def get(self, key):
        raise OSError("no params")

      def put(self, key, dat, block=True):
        raise OSError("no params")
    note_big_model(Broken(), "timed out", None, 60, False, None, None)  # nothing raised

  def test_modelds_hook_records_and_never_raises(self):
    # its own process: importing modeld sets tinygrad's environment
    code = textwrap.dedent("""
      from openpilot.common.params import Params
      from openpilot.selfdrive.modeld import modeld
      modeld.note_big_model(None, "timed out", None, 60, False, None, None)  # no params: nothing raised
      params = Params()
      params.remove("TurnPilotDiagEvents")
      modeld.note_big_model(params, "timed out", None, modeld.BIG_MODEL_TIMEOUT, True, 0.3, None)
      print(params.get("TurnPilotDiagEvents")[-1]["why"])
      params.remove("TurnPilotDiagEvents")
    """)
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=120, env=dict(os.environ))
    assert out.returncode == 0, out.stderr[-2000:]
    assert out.stdout.strip().splitlines()[-1] == "timed out after 60.0 s; the ChestnutRetry try; chestnut enumerated in 0.3 s"

  def test_the_web_apps_log_reads_and_keeps_modelds_events(self):
    params, clock = BlockingParams(), Clock()
    log = EventLog(params, clock)  # turnpilot_web, started before modeld's failure
    log.see_plan(plan("stalledTakeControl", reason="Stalled in the turn"))
    clock.t += 1
    note_big_model(params, "failed while running", raised(RuntimeError("chestnut failed with 3")), 300., False, 0.2, None, now=clock)
    assert [e["what"] for e in log.get()] == ["handback stalledTakeControl", "big model failed while running"]
    clock.t += 1
    log.see_plan(plan("cantStartTakeControl", reason="Can't start"))  # its write keeps modeld's
    assert [e["what"] for e in params.get(EVENTS_PARAM)] == ["handback stalledTakeControl", "big model failed while running",
                                                            "handback cantStartTakeControl"]
    assert [e["what"] for e in EventLog(params).get()] == [e["what"] for e in log.get()]  # and across a reboot

  def test_the_driving_model_line_says_why_and_the_chestnut_params(self):
    params, small = BlockingParams(), {"model_big": False}
    assert model_text(small, params, [])[0] == "driving model: the small model on the comma (modeld saw no chestnut when it started)"
    params.put_bool("ChestnutRetry", True, block=True)
    assert "retrying after an onroad cycle" in model_text(small, params, [])[0]
    params.put_bool("ChestnutActive", False, block=True)
    assert model_text(small, params, [])[0].endswith("(big model not running, no reason recorded)")
    note_big_model(params, "failed to load", raised(ValueError("bad pkl")), 3., True, 0.1, None)
    assert model_text(small, params, params.get(EVENTS_PARAM))[0].endswith("(big model failed to load; see events)")
    note_big_model(params, "timed out", None, 60, True, 0.1, None)
    assert model_text(small, params, params.get(EVENTS_PARAM))[0].endswith("(big model timed out after 60.0 s; see events)")
    note_big_model(params, "failed while running", raised(RuntimeError("chestnut failed with 7")), 90., True, 0.1, None)
    assert model_text(small, params, params.get(EVENTS_PARAM))[0].endswith("(big model failed while driving, retrying in the background; see events)")
    # the mid-drive recovery's own events (modeld/chestnut_recovery.py): saved like modeld's failures, read back by the report
    note_big_model_event(params, "retry failed", "attempt 1: chestnut not enumerated")
    assert params.get(EVENTS_PARAM)[-1]["what"] == "big model retry failed"
    assert model_text(small, params, params.get(EVENTS_PARAM))[0].endswith("(big model failed while driving, retrying in the background; see events)")
    note_big_model_event(params, "recovered", "big model back after 12.3 s, attempt 2, in a successor modeld")
    assert params.get(EVENTS_PARAM)[-1] | {"t": 0.} == {"t": 0., "what": "big model recovered",
                                                     "why": "big model back after 12.3 s, attempt 2, in a successor modeld"}
    assert model_text({"model_big": True}, params, params.get(EVENTS_PARAM))[0] == "driving model: the big model on the chestnut"
    assert chestnut_line(params) == "chestnut params: ChestnutLoading unset, ChestnutActive 0, ChestnutRetry 1"
    params.put_bool("ChestnutLoading", True, block=True)
    text = report(params, small, params.get(EVENTS_PARAM), [], None)
    assert "chestnut params: ChestnutLoading 1, ChestnutActive 0, ChestnutRetry 1" in text
    assert "big model failed while running: RuntimeError: chestnut failed with 7; raised after 90.0 s" in text


def gps_snapshot(**services):
  return {"procs": {"ubloxd": None, "pigeond": None, "qcomgpsd": True}, "services": services}


def qcom_no_fix():
  return {"seen": True, "up": True, "valid": True, "fix": False, "age_s": 0.4, "source": "qcomdiag", "satellites": 0,
          "h_acc": 0., "v_acc": 500., "speed_acc": 0.31, "bearing_acc": 180.}


class TestGps(OpenpilotTestCase):
  """The GPS section: which daemon runs, each GPS message's state and quality, qcomgpsd's assistance and LastGPSPosition,
  never a coordinate."""
  def test_a_qcom_device_without_a_fix(self):
    params = BlockingParams()
    text = "\n".join(diagnostics.gps_text(gps_snapshot(gpsLocation=qcom_no_fix(), gpsLocationExternal={"seen": False}),
                                          params, False, True, 1_761_940_000.))
    assert "daemon by process_config's rule: qcomgpsd (no /dev/ttyHS0, or use-quectel-gps); UbloxAvailable unset" in text
    assert "running: ubloxd not managed, pigeond not managed, qcomgpsd running" in text
    assert ("gpsLocation: publishing, valid yes, hasFix no, age 0.4 s, source qcomdiag, satellites not reported, " +
            "vertical accuracy 500.0 (qcomgpsd's no-fix value), speed accuracy 0.31 m/s, bearing accuracy 180 deg") in text
    assert "horizontal accuracy" not in text  # qcomgpsd never sets it
    assert "gpsLocationExternal: never received since the web app started" in text
    assert "downloads no assistance data" in text and "clock valid now: yes" in text
    assert "LastGPSPosition: none" in text

  def test_a_ublox_device_with_a_fix_and_a_saved_position(self):
    params = BlockingParams()
    params.put_bool("UbloxAvailable", True)
    msg = messaging.new_message("gpsLocationExternal", valid=True)
    msg.gpsLocationExternal.latitude, msg.gpsLocationExternal.longitude = 39.768403, -86.158102
    msg.gpsLocationExternal.unixTimestampMillis = int((1_761_940_000. - 3 * 3600) * 1e3)
    params.put("LastGPSPosition", msg.to_bytes())
    ext = {"seen": True, "up": True, "valid": True, "fix": True, "age_s": 0.1, "source": "ublox", "satellites": 11,
           "h_acc": 2.4, "v_acc": 3.1, "speed_acc": 0.12, "bearing_acc": 4.}
    gps = {"procs": {"ubloxd": True, "pigeond": True, "qcomgpsd": None}, "services": {"gpsLocationExternal": ext}}
    text = report(params, None, [], [], "12.8", now=1_761_940_000., gps=gps, ublox=lambda: True, clock=lambda: True)
    assert "daemon by process_config's rule: ubloxd and pigeond (/dev/ttyHS0 is there); UbloxAvailable 1" in text
    assert "gpsLocationExternal: publishing, valid yes, hasFix yes, age 0.1 s, source ublox, satellites 11, horizontal accuracy 2.4 m" in text
    assert "gpsLocation: never received" in text and "assistance" not in text
    assert "LastGPSPosition: present (gpsLocationExternal, 3 h old)" in text
    for pos in ("39.7", "86.1", "39.768403", "-86.158102"):
      assert pos not in text, pos

  def test_no_reader_or_manager(self):
    text = "\n".join(diagnostics.gps_text(None, BlockingParams(), None, None, 0.))
    assert "daemon by process_config's rule: unknown" in text and "status reader not running" in text and "clock valid now: unknown" in text
    text = "\n".join(diagnostics.gps_text({"procs": {}, "services": {}}, BlockingParams(), False, False, 0.))
    assert "running: unknown (managerState isn't publishing)" in text and "clock valid now: no" in text

  def test_the_rule_is_process_configs(self):
    from openpilot.system.manager import process_config
    for present, want in (({"/dev/ttyHS0"}, True), ({"/dev/ttyHS0", "/persist/comma/use-quectel-gps"}, False), (set(), False)):
      with mock.patch.object(os.path, "exists", lambda p, present=present: p in present):
        assert diagnostics.gps_daemon_rule() is want is process_config.ublox_available()


class TestQcomSavedPosition(OpenpilotTestCase):
  """qcomgpsd saves its fix to LastGPSPosition as ubloxd does, a gpsLocationExternal event, so the phone app's offroad
  position and pigeond read it unchanged."""
  def fix(self):
    msg = messaging.new_message("gpsLocation", valid=True)
    gps = msg.gpsLocation
    gps.latitude, gps.longitude, gps.hasFix = 39.768403, -86.158102, True
    gps.unixTimestampMillis, gps.source = 1_761_940_000_000, log.GpsLocationData.SensorSource.qcomdiag
    return msg

  def test_the_store_reads_it_and_the_source_stays(self):
    params = BlockingParams()
    qcomgpsd.save_position(params, self.fix())
    assert last_gps_position(params) == (39.768403, -86.158102)
    with log.Event.from_bytes(params.get("LastGPSPosition")) as evt:
      assert evt.which() == "gpsLocationExternal"  # what pigeond's send_position reads
      assert evt.gpsLocationExternal.source == log.GpsLocationData.SensorSource.qcomdiag
      assert evt.gpsLocationExternal.unixTimestampMillis == 1_761_940_000_000
    assert "present (gpsLocationExternal" in diagnostics.saved_position_text(params, 1_761_940_060.)

  def test_a_params_failure_never_raises(self):
    class Broken:
      def put(self, *a):
        raise OSError("read-only")
    qcomgpsd.save_position(Broken(), self.fix())


def test_radio_text_says_what_the_ublox_hears():
  from openpilot.selfdrive.turnpilot.diagnostics import radio_text
  def now(): return 100.
  assert radio_text({}) == ["  u-blox radio: no ubloxGnss messages read"]
  lines = radio_text({"hw": {"noise": 95, "agc": 3432, "antenna": "ok", "antenna_power": "on", "jamming": "12", "at": 99.},
                      "sats": {"listed": 20, "heard": 0, "best_cno": 0, "at": 98.}, "meas": {"count": 0, "at": 98.}}, now=now)
  assert lines[0].startswith("  u-blox radio: noise 95 per ms, AGC 3432 of 8191, antenna ok")
  assert lines[1] == "  u-blox satellites: hears 0 of 20 it expects, best signal 0 dB-Hz (2 s ago)"
  assert lines[2] == "  u-blox measurements: 0 (2 s ago)"


def test_see_ublox_reads_each_ublox_message_and_never_a_position():
  from openpilot.cereal import messaging
  from openpilot.selfdrive.turnpilot.web.status import see_ublox
  hw = messaging.new_message("ubloxGnss")
  h = hw.ubloxGnss.init("hwStatus")
  h.noisePerMS, h.agcCnt = 95, 3432
  sat = messaging.new_message("ubloxGnss")
  svs = sat.ubloxGnss.init("satReport").init("svs", 3)
  svs[0].cno, svs[1].cno, svs[2].cno = 0, 31, 0
  radio: dict = {}
  see_ublox(radio, [hw.as_reader(), sat.as_reader()])
  assert radio["hw"]["noise"] == 95 and radio["hw"]["agc"] == 3432
  assert radio["sats"]["listed"] == 3 and radio["sats"]["heard"] == 1 and radio["sats"]["best_cno"] == 31
  assert "latitude" not in str(radio) and "longitude" not in str(radio)
