"""The vehicle-CAN cable as the docs name it (docs/turnpilot: INSTALL.md, SHADOW_CHECKLIST.md) against car_preflight's X179
add-on fix (VEHICLE_CAN_CABLE, VEHICLE_CAN_PORT, VEHICLE_CAN_LISTED) and the sources the docs cite for it.

Review 2026-11-07: the docs, the checklist and car_preflight's fix sent the owner to "Tesla's X179 connector, behind the
right A-pillar trim", with model years from "Tesla's guides", and cited nothing the repo could check; INSTALL.md said the
bus-1 path "has never run on a car", where opendbc tesla/car_signal.py reports lamps lit from a 0x3E9 sent on bus 1 on a
real car (carrotpilot, 2026-10-23). The sources: xnor's listing ("CAN debug port in the passenger-side A-pillar", "Built
after 2021", left-hand drive only, "verify your vehicle has this port before ordering") and Tesla's Model 3 EDR guide for
cars built after 10/04/2020 (CD-20-20-003: a CAN connector in the passenger footwell, the right A-pillar trims off and the
carpet folded back, never named X179; cars built through that date have their own guide)."""
import re
from pathlib import Path

from openpilot.tools.turnpilot import car_preflight as cpf

DOCS = Path(__file__).parents[4] / 'docs' / 'turnpilot'
LISTING = "https://xnor.shop/products/model-3-y-hw3-hw4-vehicle-can-addon-cable"
TESLA_EDR_GUIDE = "CD-20-20-003"
PHRASES = (cpf.VEHICLE_CAN_CABLE, cpf.VEHICLE_CAN_PORT, cpf.VEHICLE_CAN_LISTED)


def section(doc: str, heading: str) -> str:
  text = (DOCS / doc).read_text()
  start = text.index(f"\n## {heading}\n")
  end = text.find("\n## ", start + 1)
  return text[start:end if end >= 0 else None]


def cable_paragraphs(doc: str) -> list[str]:
  return [p for p in (DOCS / doc).read_text().split("\n\n") if "Vehicle CAN Addon Cable" in p]


def test_install_cites_the_cables_listing_and_teslas_guide_to_its_port():
  text = section("INSTALL.md", "Let the car signal its own turns")
  assert f"({LISTING})" in text
  assert re.search(rf"\[{TESLA_EDR_GUIDE}\]\(https://service\.tesla\.com/[^)]*{TESLA_EDR_GUIDE}[^)]*\.pdf\)", text)
  assert "doesn't name the connector X179" in text and "have a guide of their own" in text


def test_the_docs_name_the_cable_its_port_and_its_listing_as_car_preflight_does():
  """INSTALL.md's and the checklist's cable paragraphs and car_preflight's fix agree word for word on the cable, where it
  plugs in and what it is listed for, so a fix to one that misses another goes red here."""
  for doc in ("INSTALL.md", "SHADOW_CHECKLIST.md"):
    paragraphs = cable_paragraphs(doc)
    assert paragraphs, doc
    for p in paragraphs:
      for phrase in PHRASES:
        assert phrase in p, (doc, phrase)
      assert "X179" in p, doc  # the port car_preflight's line is named for, said so
  for phrase in PHRASES:
    assert phrase in cpf.VEHICLE_CAN_FIT


def test_no_doc_calls_x179_teslas_or_says_the_bus_1_path_never_ran():
  """Tesla's guide never names the port X179, and car_signal.py's one real-car report of lit lamps sent its copy on bus 1."""
  for doc in ("INSTALL.md", "SHADOW_CHECKLIST.md"):
    for p in cable_paragraphs(doc):
      assert "Tesla's X179" not in p and "Tesla's guides place X179" not in p, doc
      assert "never run on a car" not in p, doc
  text = section("INSTALL.md", "Let the car signal its own turns")
  assert "TurnPilot's own bus-1 path hasn't run yet" in text and "carrotpilot, 2026-10-23" in text
