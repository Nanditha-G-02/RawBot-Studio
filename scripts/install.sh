#!/usr/bin/env bash
# PHYSIOS installation. Run once after cloning (safe to run again).
#
#   ./scripts/install.sh                 check requirements, create the configuration and the
#                                        data folder, build the frontend
#   ./scripts/install.sh --docker        also build the gateway image
#   ./scripts/install.sh --robot         also build the robot runtime image (large: ROS 2,
#                                        LeRobot; needs Docker, ~20 GB disk, 20-40 min)
#   ./scripts/install.sh --with-mujoco   also install the MuJoCo physics engine for the simulated
#                                        robot, in its own Python environment (var/sim-venv)
#   ./scripts/install.sh --import-from DIR
#                                        copy datasets, plans, runs and checkpoints from an
#                                        earlier installation (see scripts/import-legacy-data.py)
# Nothing here moves the robot or starts a service; start PHYSIOS with ./start_physios.sh.
set -euo pipefail
PHYSIOS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER=0; ROBOT=0; MUJOCO=0; IMPORT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --docker) DOCKER=1 ;;
    --robot) ROBOT=1 ;;
    --with-mujoco) MUJOCO=1 ;;
    --import-from) IMPORT="${2:?--import-from needs a folder}"; shift ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)"; exit 2 ;;
  esac
  shift
done
ok()   { printf '  \e[32m✓\e[0m %s\n' "$*"; }
warn() { printf '  \e[33m!\e[0m %s\n' "$*"; }
fail() { printf '  \e[31m✗\e[0m %s\n' "$*"; exit 1; }
echo "RawBot Studio $(cat "$PHYSIOS/VERSION") - installing in $PHYSIOS"

echo "requirements"
PY="${PHYSIOS_PYTHON:-$(command -v python3 || true)}"
[[ -n "$PY" ]] || fail "python3 not found (Python 3.9 or newer)"
"$PY" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)' || fail "Python 3.9+ needed ($("$PY" -V))"
ok "$("$PY" -V): gateway and training service (standard library only, no packages)"
if command -v node >/dev/null && command -v npm >/dev/null; then
  node -e 'process.exit(parseInt(process.versions.node) >= 18 ? 0 : 1)' || fail "Node.js 18+ needed to build the frontend ($(node -v))"
  ok "Node.js $(node -v): frontend build"
elif [[ -f "$PHYSIOS/frontend/dist/index.html" ]]; then
  warn "Node.js not found; using the frontend already built in frontend/dist"
else
  fail "Node.js 18+ and npm are needed to build the frontend (https://nodejs.org or: sudo apt install nodejs npm)"
fi
if command -v docker >/dev/null && docker compose version >/dev/null 2>&1; then
  ok "$(docker compose version | head -1): robot runtime and Docker deployment"
else
  warn "Docker with the compose plugin not found: needed for the robot runtime (recording, inference) and for the Docker deployment"
fi

echo "configuration and data"
CFG="$PHYSIOS/configuration/physios.env"
if [[ -f "$CFG" ]]; then ok "configuration/physios.env exists (kept)"
else cp "$PHYSIOS/configuration/physios.env.example" "$CFG"; ok "configuration/physios.env created from the example"; fi
set -a; # shellcheck disable=SC1090
source "$CFG"; set +a
DATA="${PHYSIOS_DATA_DIR:-$PHYSIOS/data}"
mkdir -p "$DATA"/{huggingface/lerobot,workspace/physios,outputs/train,robot-config} "$PHYSIOS/var"
ok "data folder: $DATA"

if [[ -n "$IMPORT" ]]; then
  echo "importing from $IMPORT"
  rc=0; "$PY" "$PHYSIOS/scripts/import-legacy-data.py" "$IMPORT" --data "$DATA" || rc=$?
  if [[ $rc == 3 ]]; then warn "some old files belong to root: run the sudo command above, then carry on"
  elif [[ $rc != 0 ]]; then fail "import failed (exit $rc)"; fi
fi

echo "frontend"
if command -v npm >/dev/null; then
  ( cd "$PHYSIOS/frontend" && npm ci --no-audit --no-fund && npm run build ) >"$PHYSIOS/var/install-build.log" 2>&1 \
    || fail "frontend build failed, see var/install-build.log"
  ok "built: frontend/dist"
fi
( cd "$PHYSIOS/backend" && "$PY" -m physios_gateway --check-config >/dev/null ) && ok "gateway configuration valid" \
  || fail "configuration invalid: run (cd backend && python3 -m physios_gateway --check-config)"

if [[ $MUJOCO == 1 ]]; then
  # the training service stays standard-library only; MuJoCo runs in this separate Python
  # (hardware/mujoco_worker.py), found through PHYSIOS_SIM_PYTHON or var/sim-venv
  echo "MuJoCo physics"
  SIMPY="$PHYSIOS/var/sim-venv/bin/python"
  if [[ ! -x "$SIMPY" ]]; then
    "$PY" -m venv "$PHYSIOS/var/sim-venv" >"$PHYSIOS/var/install-mujoco.log" 2>&1 \
      || fail "could not create var/sim-venv (Ubuntu: sudo apt install python3-venv), see var/install-mujoco.log"
  fi
  "$SIMPY" -m pip install --upgrade --quiet pip mujoco >>"$PHYSIOS/var/install-mujoco.log" 2>&1 \
    || fail "pip install mujoco failed, see var/install-mujoco.log"
  ok "MuJoCo $("$SIMPY" -c 'import mujoco; print(mujoco.__version__)'): Planning -> Robot & control -> Engine: MuJoCo"
fi
if [[ $DOCKER == 1 ]]; then echo "gateway image"; "$PHYSIOS/docker/physios-docker.sh" build gateway; fi
if [[ $ROBOT == 1 ]]; then echo "robot runtime image"; "$PHYSIOS/docker/physios-docker.sh" build robot; fi

cat <<MSG

RawBot Studio is installed.
  start:        ./start_physios.sh                (gateway + training service)
  with robot:   ./start_physios.sh --with-robot   (also the robot runtime container)
  in Docker:    ./docker/physios-docker.sh up [--with-robot]
  open:         http://127.0.0.1:${PHYSIOS_PORT:-8700}
MSG
