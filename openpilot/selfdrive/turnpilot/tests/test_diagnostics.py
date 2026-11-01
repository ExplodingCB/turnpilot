"""The diagnostics report Chase copies from the phone page for Kellen (diagnostics.py): what it says, and that it never says
the map key, the web token or a position."""
import json
import os
import subprocess
import sys
import textwrap

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot import diagnostics, preflight
from openpilot.selfdrive.turnpilot.diagnostics import (BIG_WHY_MAX, EVENTS_KEPT, EVENTS_PARAM, REPEAT_S, EventLog, Preflight, chestnut_line,
                                                        log_event, model_text, note_big_model, report)

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
    assert model_text(small, params, params.get(EVENTS_PARAM))[0].endswith("(big model failed while driving, fell back; see events)")
    assert model_text({"model_big": True}, params, params.get(EVENTS_PARAM))[0] == "driving model: the big model on the chestnut"
    assert chestnut_line(params) == "chestnut params: ChestnutLoading unset, ChestnutActive 0, ChestnutRetry 1"
    params.put_bool("ChestnutLoading", True, block=True)
    text = report(params, small, params.get(EVENTS_PARAM), [], None)
    assert "chestnut params: ChestnutLoading 1, ChestnutActive 0, ChestnutRetry 1" in text
    assert "big model failed while running: RuntimeError: chestnut failed with 7; raised after 90.0 s" in text
