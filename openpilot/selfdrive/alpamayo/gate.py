"""Who owns the chestnut. Alpamayo and the big driving model (and so TurnPilot, which needs it) can't share it:
tinygrad holds an exclusive lock per device."""
from openpilot.common.params import Params


def alpamayo_mode(params: Params) -> str:
  mode = (params.get("AlpamayoMode") or "off").strip().lower()
  if mode not in ("off", "shadow", "active") or params.get_bool("TurnPilotEnabled"):
    return "off"  # TurnPilot wins: it needs the big model on the chestnut
  return mode


def alpamayo_owns_chestnut(params: Params) -> bool:
  return alpamayo_mode(params) != "off"
