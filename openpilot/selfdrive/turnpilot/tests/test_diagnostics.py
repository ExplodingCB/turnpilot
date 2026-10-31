"""The diagnostics report Chase copies from the phone page for Kellen (diagnostics.py): what it says, and that it never says
the map key, the web token or a position."""
import json

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot import diagnostics, preflight
from openpilot.selfdrive.turnpilot.diagnostics import EVENTS_KEPT, EVENTS_PARAM, REPEAT_S, EventLog, Preflight, log_event, report

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
