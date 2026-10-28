"""TurnPilot's bounded go (go_override.py; DESIGN phase 4 decision 1 and its stop-sign amendment, G5): plannerd lifts the
e2e model's hold on a standing car only for a go turnpilotd certifies, at most +0.5 m/s^2, for at most 3 s, under 3 m/s,
and only while nothing else holds the car."""
from openpilot.cereal import custom, log


def test_schema_carries_the_certified_go_and_the_lift():
  # appended only: TurnpilotPlan @32 after unprotectedLeft @31, LongitudinalPlan @41-@42 after accelBoost @40
  causes = list(custom.TurnpilotPlan.GoCertified.Cause.schema.enumerants)
  assert causes == ['none', 'turnRelease', 'unprotectedLeft', 'stopSign', 'rollUp', 'creep', 'clearBox']
  fields = custom.TurnpilotPlan.schema.fields
  assert fields['goCertified'].proto.ordinal.explicit == 32
  assert list(custom.TurnpilotPlan.GoCertified.schema.fields) == ['cause', 'id', 't']
  lp = log.LongitudinalPlan.schema.fields
  assert lp['turnpilotGoLift'].proto.ordinal.explicit == 41 and lp['turnpilotGoEnd'].proto.ordinal.explicit == 42
