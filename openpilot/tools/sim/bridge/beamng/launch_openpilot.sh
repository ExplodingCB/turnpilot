#!/usr/bin/env bash
# openpilot as a comma four in a Tesla Model 3, fed by the BeamNG bridge (run_bridge.py). Like tools/sim/launch_openpilot.sh,
# but a Model 3 with alpha longitudinal instead of a Civic, and loggerd and logmessaged left on so every run leaves an rlog
# with cloudlog's records in it.
# OPENPILOT_ROOT picks the (built) checkout to run; extra processes to block go in BLOCK, e.g. BLOCK=turnpilotd for gate1.py.

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="TESLA_MODEL_3"

# the bridge is camerad; encoderd writes lossless video on PC, too heavy for the Mac
export BLOCK="${BLOCK},camerad,encoderd,micd,manage_athenad"
# logmessaged runs with loggerd: it publishes cloudlog's records as logMessage, which loggerd writes to the rlog, so a run's
# rlog says why TurnPilot did what it did (turnpilot.go_not_certified, ...). Upstream's sim blocks it with loggerd, having
# no rlog to carry them; a BLOCK that leaves loggerd out here leaves logmessaged out with it, for the same reason
if [[ ",${BLOCK}," == *",loggerd,"* ]]; then
  export BLOCK="${BLOCK},logmessaged"
fi
if [[ "$CI" ]]; then
  export BLOCK="${BLOCK},ui"
fi

# msgq needs the prefix directory to exist (macOS keeps msgq in /tmp)
if [[ -n "$OPENPILOT_PREFIX" ]]; then
  mkdir -p "/tmp/msgq_$OPENPILOT_PREFIX"
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
OPENPILOT_ROOT="${OPENPILOT_ROOT:-$DIR/../../../../..}"
cd "$OPENPILOT_ROOT"
export PYTHONPATH="$OPENPILOT_ROOT${PYTHONPATH:+:$PYTHONPATH}"

# navd routes on the sim map's own tiles, never on real roads (the default 'auto' falls back to Mapbox);
# TURNPILOT_TILES picks the tiles, e.g. ~/turnpilot_data/valhalla/west_coast_usa_lanes for the turn-lane demos. Without it,
# the tiles with the level's stop signs and signals (tools/turnpilot/build_sim_tiles.sh) when they are built.
# No gas go (2026-10-28): any gas press disengages, as on the car (DisengageOnAccelerator on, preflight fails without it).
# TURNPILOT_LIGHT_ORACLE=1 lets turnpilotd take the sim's light oracle (run_bridge.py --light-oracle); SIM ONLY, it needs
# SIMULATION=1 too, and the param clears on every manager start.
python3 -c "
import os
from openpilot.common.params import Params
from openpilot.selfdrive.test.helpers import set_params_enabled
set_params_enabled()
p = Params()
p.put_bool('AlphaLongitudinalEnabled', True)
p.put_bool('DisengageOnAccelerator', True)
p.put('TurnPilotRouteSource', 'valhalla')
signals = os.path.expanduser('~/turnpilot_data/valhalla/west_coast_usa_signals')
default = signals if os.path.exists(os.path.join(signals, 'controls_built')) else os.path.expanduser('~/turnpilot_data/valhalla/west_coast_usa')
tiles = os.environ.get('TURNPILOT_TILES') or p.get('TurnPilotValhallaTiles') or default
p.put('TurnPilotValhallaTiles', tiles)
print(f'TurnPilotValhallaTiles: {tiles}')
"

# after the manager clears TurnPilotSimLightOracle at its start (CLEAR_ON_MANAGER_START)
if [[ "$TURNPILOT_LIGHT_ORACLE" == "1" ]]; then
  (sleep 5 && python3 -c "from openpilot.common.params import Params; Params().put_bool('TurnPilotSimLightOracle', True)" \
    && echo "TurnPilotSimLightOracle set: turnpilotd takes the sim's light oracle") &
fi

cd openpilot/system/manager && exec ./manager.py
