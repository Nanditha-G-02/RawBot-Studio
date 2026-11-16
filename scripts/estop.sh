#!/usr/bin/env bash
# RawBot Studio emergency stop from a terminal - works without any browser page.
#
#   ./scripts/estop.sh
#
# Stops everything RawBot Studio commands: the follower jog session and a host policy
# rollout get a stop and are killed 0.5 s later; a running calibration is cancelled (the
# original motor values are written back). If the training service does not answer, the
# worker processes are killed directly.
#
# Motion that bridge.py produces itself (teleoperation) is not RawBot Studio's: for that,
# and always when in doubt, switch the arm's power off.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${PHYSIOS_CONFIG:-$HERE/configuration/physios.env}"
[[ -f "$CONFIG" ]] && { set -a; # shellcheck disable=SC1090
  source "$CONFIG"; set +a; }
TRAINER_URL="${PHYSIOS_TRAINER_URL:-http://127.0.0.1:8766}"
PY="${PHYSIOS_PYTHON:-python3}"

if "$PY" - "$TRAINER_URL" <<'PYEOF'
import json, sys, urllib.request
req = urllib.request.Request(sys.argv[1].rstrip('/') + '/hardware/estop',
                             data=json.dumps({'source': 'command line'}).encode(),
                             headers={'Content-Type': 'application/json'})
try:
    r = json.load(urllib.request.urlopen(req, timeout=3))
except Exception as e:  # noqa: BLE001
    print('training service not reachable: %s' % e)
    sys.exit(1)
print('emergency stop sent: %s' % (', '.join(r.get('stopped') or []) or 'nothing was moving'))
PYEOF
then
  exit 0
fi
echo "Killing RawBot Studio motion workers directly..."
pkill -KILL -f 'physios_trainer/hardware/jog_worker.py' && echo "  jog worker killed"
pkill -KILL -f 'physios_trainer/rollout_worker.py run' && echo "  rollout worker killed"
pkill -TERM -f 'physios_trainer/hardware/hw_worker.py calibrate' && echo "  calibration stopped (values NOT restored - re-calibrate)"
echo "Done. If the arm is still moving, switch its power off."
