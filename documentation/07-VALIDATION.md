# RawBot Studio 2.0.0: validation report

Date: 13 November 2026 · Repository `github.com/Nanditha-G-02/physios`, version 2.0.0.

Environment: cloud sandbox (Ubuntu 24.04, Python 3.13, Node 22, Chromium via Playwright),
**no robot, no GPU, no ROS**. All checks ran from a fresh clone, in a folder with no
`physical_ai_tools` checkout anywhere.

This report checks 2.0.0 against the instruction of 13 Nov. RawBot Studio must be a completely
independent project with its own repository, backend, frontend, configuration, dependencies,
Docker setup and execution workflow. It must not need the ROBOTIS `physical_ai_tools`
repository or its `physical_ai_server` / `physical_ai_trainer` backends. Simulated tests
are **not** a claim of completion on hardware; section 4 lists what must be checked on the
robot PC.

## 1. Requirements traceability

| Requirement | Where it is met | Evidence | Status |
|---|---|---|---|
| Own repository, installable from it alone | `git clone` + `scripts/install.sh` | fresh-clone install, start-up, tests (section 2) | met |
| No `physical_ai_tools` checkout needed | nothing refers to a parent folder; no `PHYSIOS_REPO_ROOT` | `test_repository_is_self_contained` | met |
| Own backend instead of `physical_ai_trainer` | `backend/physios_trainer` (the v4 training service, maintained in RawBot Studio) | 91 + 1 skipped unchanged tests; started by `start_physios.sh` | met |
| Own robot backend instead of the ROBOTIS `physical_ai_server` container | `robot/src` (robot server incl. planning engine, interfaces, recorder) + `docker/robot` image `physios-robot` | planning tests 45; Dockerfile check; compose config. Image build and real robot: section 4 | met in code; build and hardware to confirm |
| Own frontend | `frontend/`, built by `install.sh`, served by the gateway | CI build; 26/26, 9/9, 6/6 | met |
| Own configuration | `configuration/physios.env` (`PHYSIOS_DATA_DIR`, robot runtime settings) | `--check-config`; install creates it | met |
| Own data storage | `data/` (`PHYSIOS_DATA_DIR`), shared by all parts at fixed paths | standalone tests; trainer reports `data_root`, container paths | met |
| Own dependencies | host: Python stdlib, Node for the build; robot image: ROS 2 Jazzy, LeRobot pinned in `robot/lerobot.lock` (upstream Hugging Face) | `install.sh` checks; lock test | met |
| Own Docker setup | `docker/gateway`, `docker/robot`, `docker-compose.yml` (+ GPU override), `physios-docker.sh` | gateway container healthy; robot image to build on the PC | met (robot image: section 4) |
| Own execution workflow | `start_physios.sh [--with-robot] [--no-gateway]`, `stop_physios.sh [--robot]`, `physios-docker.sh up [--with-robot]` | start/stop from the fresh clone | met |
| Keep existing data | `scripts/import-legacy-data.py` (copy, hard-link, path rewrite; old checkout untouched) | `test_import_legacy_data` | met (real data: section 4) |
| Keep the interface and the workflow | same pages; instruction texts and container paths updated only | walk-through 26/26 | met |
| No automatic motion at start-up | scripts and containers start services only | code review of `start_physios.sh` and `docker/robot/entrypoint.sh` | met |
| Third-party code credited | `NOTICE`, original headers kept | — | met |

## 2. Automated test results (13 Nov 2026, fresh clone of 2.0.0)

| Suite | Command | Result |
|---|---|---|
| Installation | `./scripts/install.sh` | completed (config, data folder, frontend build, config check) |
| Gateway | `cd backend && python3 -m pytest -q tests` | **22 passed** |
| Training service + standalone layout | `cd backend && python3 -m pytest -q trainer_tests` | **102 passed, 1 skipped** |
| Planning engine | `cd robot/src/physical_ai_server && python3 -m pytest -q test/planning` | **45 passed** |
| Frontend production build | `cd frontend && CI=true npm run build` | **compiled** |
| Whole workflow | `tests/e2e/run_e2e.py http://127.0.0.1:8700` against `tests/e2e/devstack.py` | **26/26** |
| Optimize tab | `tests/e2e/opt_ui_check.py` against `devstack.py --seed` | **9/9** |
| Sign-in | `tests/e2e/auth_e2e.py` | **6/6** |

### Flexible robot setup (16 Nov 2026)

Against simulated hardware: a fake Linux sysfs with two USB serial adapters, simulated Feetech
STS3215 motor buses, a stand-in `bridge.py` holding the ports and a stand-in teleoperation
topic (`devstack.py --hardware bridge`).

| Scenario | Unit tests (`trainer_tests/test_hardware.py`) | Browser (`tests/e2e/hw_ui_check.py`) |
|---|---|---|
| 1. Leader + Follower | both arms through bridge.py, teleop and recording on | cards connected, capabilities on |
| 2. Leader Only | follower unused, leader features only | leader capture saved, Record Start shows "Needs Leader + Follower" |
| 3. Follower Only | jog, hand-guided recording, policy runs | jog session started and ended; Record shows "Hand-guided" and Start is allowed |
| 4. Simulation Only | nothing uses an arm; never changed automatically | Record and Inference Start show "Simulation Only is selected" |
| 5. Leader lost during operation | unplug and stale messages turn it *lost* | Record Start disabled with the reason, a message appears |
| 6. Follower lost during operation | unplugged during a read-only session | Inference Start disabled with the reason |
| 7. Reconnection | same adapter on a new port is recognised; ROS arm comes back | plugged back in, recording on again |
| 8. Independent calibration | profile saved per arm; cancel restores the motors; refused while bridge.py holds the port | follower connected alone (read-only), calibrated, leader untouched |
| 9. Dynamic availability | a second arm appearing later switches to Leader + Follower; an arm present when the operator chooses does not | leader plugged in later switches Follower Only to Leader + Follower |

Results: `test_hardware.py` **22 passed**; `hw_ui_check.py` **9/9**. The 26-step workflow
and the Optimize check still pass with the simulated arms present.

**Not validated:** the real SO-101 arms, the real `bridge.py` (its topic names and whether it
publishes the leader topic only during teleoperation), the real LeRobot Feetech bus (the
worker uses LeRobot's `FeetechMotorsBus` API as in LeRobot 0.6/0.7), and hand-guided recording
in the rebuilt robot image.

### Guided robot setup (16 Nov 2026)

`tests/e2e/setup_flow_check.py` **10/10**: a built-in robot from Home through hardware (with
Simulation Only skipping the arm checks), verification and the summary into Planning; a custom
robot started from its model ZIP (file picker first, name taken from the URDF, details saved
afterwards, Stage 1 registration, hardware choice, verification holding Planning until the
later stages are done), and the flow in dark mode. The 26-step walk-through (now through the
flow), `hw_ui_check.py` 9/9 and the Optimize check pass.

### Hardware-aware Planning (16 Nov 2026)

`trainer_tests/test_planning_setup.py` **14 passed**: the choices and recording decision for
Leader + Follower, Leader Only (with and without the simulated twin), Follower Only (hand guiding,
direct control), Simulation Only (cameras must publish; unknown publishers do not block),
Physical + Simulated (mirror, comparison, leader drives the simulation); a stored choice that no
longer fits falls back to the configuration default and is reset when the configuration changes;
the simulated robot's joint speed, dead-man, scripts, replays, captures, and that it yields the
follower topic to a real publisher. `test/planning` **46 passed** (a take keeps how it was
recorded).

`tests/e2e/plan_hw_check.py` **6/6** (then) in the browser: the five configurations above with their
controls (sliders, held keyboard keys, waypoint script, dataset and capture replay, trajectory
capture, keyboard direct control of the follower), the Record page control, a plan episode
recorded with the simulated robot keeping its setup, both themes. `hw_ui_check.py` 9/9 (two
expectations updated: Leader Only now points to the simulated twin; Simulation Only records the
simulated robot), `setup_flow_check.py` 10/10, the 26-step walk-through with simulated arms and
with `--hardware none`.

**Not validated:** recording the simulated robot through the real physical_ai_server (the
stand-in recorder writes synthetic joint data), a real gamepad, the real `bridge.py` together
with the simulated robot on one ROS graph.

### Physical Leader + Simulated Follower (17 Nov 2026)

`test_planning_setup.py` (now 20): tick-to-radian conversion and joint map; a leader read over
serial (simulated Feetech bus) drives the simulated follower including the gripper; the leader
joints are published on `/rawbot/leader/joint_states` and never on the command topic; joint limits
clamp and are reported; an older robot runtime, a stale or disconnected leader block recording;
with bridge.py the leader topic is used; no automatic switch to Leader + Follower. Also: a jog
Stop can no longer be overwritten by a concurrent heartbeat (found while testing).
`plan_hw_check.py` **7/7** (new P6: Home → configuration, serial connect, the motor moved,
simulated follower at +π/2 and the gripper following, Record, a plan take saved with
`action_param leader_state`, dark mode). `hw_ui_check.py` 9/9, `setup_flow_check.py` 10/10,
26-step walk-through, Optimize: pass. trainer_tests 145 (1 skipped), planning 46, gateway 22.

**Not validated:** the new `leader_state` path in the real physical_ai_server (it cannot run in
the sandbox; compiled only), your real leader arm's tick convention against your dataset.

### Robot-aware Planning (17 Nov 2026)

`test_planning_setup.py` 24: capabilities from joint names alone (8 joints with a finger: keyboard
yes, gamepad no); a custom 3-joint robot with a prismatic gripper from its URDF (ROS names, soft
limits shaping the simulated robot, gamepad allowed); a leader missing the gripper blocks
recording; only a leader in Leader + Follower uses the simulated follower, a set-up but unplugged
follower does not. `plan_hw_check.py` 7/7 (P3 now: Leader Only → automatic simulated follower,
checks green, follower option hidden), `hw_ui_check.py` 9/9 (Leader Only expectation updated),
`setup_flow_check.py` 10/10, 26-step walk-through with arms and with `--hardware none`, Optimize.
trainer_tests 149 (1 skipped), planning 46, gateway 22.

**Not validated:** a real custom robot end to end (its robot type must be active on the robot
server), a robot with more than 6 joints in the browser.

### Physical follower detection (17 Nov 2026)

Root causes found in the code: Detect skipped held / permission-denied ports without recording a
reason; port holders of other users (ModemManager, brltty, container processes) are invisible to
the /proc scan, so the scan failed with a raw serial error; nothing published the follower's joint
states on ROS without bridge.py; "ROS not reachable" was shown before a connection was even tried.
Fixed: every port gets a reason, serial errors are explained, RawBot Studio relays its serial
readings on the follower topic when nothing else does (`relay.py`), the ROS line says *not checked
yet* until a connection is attempted. New `scripts/hw-doctor.sh`, run against the simulated
adapters (free, held by a stand-in bridge.py, three motors missing) and the dev stack.
`test_planning_setup.py` 27; trainer_tests 152 (1 skipped); browser checks unchanged and passing.

**Not validated:** your PC (ModemManager / brltty / dialout state, the real arm's motors), the
relay feeding the real physical_ai_server.

### Architecture phases 3, 5 (part) and mixed robots (17 Nov 2026)

Built from the *System Architecture & Scenario Matrix*: scenario A3 (simulated leader → physical
follower, `jog_worker` follow mode), joint pairs for robots with different joint names, and the
recording metadata in the dataset's `meta/`. `test_planning_setup.py` 30 (follow set-up, line-up on
start and on switch-on, missing simulated joints refused, mixed-robot pairs); `test/planning` 47
(meta files written, `info.json` untouched); `plan_hw_check.py` **8/8** (P7: follow with limits
review, the stand-in follower tracks the simulated leader, a plan episode recorded and its
metadata line checked, pause, end); `hw_ui_check.py` 9/9, `setup_flow_check.py` 10/10, 26-step
walk-through with arms and with `--hardware none`, Optimize. trainer_tests 155 (1 skipped).
Also fixed while testing: a page coming back to a follow session showed it as paused.

**Not validated:** follow mode on the real arm (bridge.py executing the stream), the metadata
files with the real recorder.

## 3. Deployment validation

| Path | What was run | Result |
|---|---|---|
| Scripts | `./start_physios.sh` / `./stop_physios.sh` from the fresh clone | gateway and training service started from `backend/`; health lists `physios_trainer 2.0.0`; stopped cleanly |
| Docker, gateway | `./docker/physios-docker.sh up` with `PHYSIOS_BASE_IMAGE` = a locally available Ubuntu 24.04 + Python image | container *healthy*; data folder at `/physios/data` (read-only); dataset roots as documented |
| Docker, robot runtime | `docker build --check` of `docker/robot/Dockerfile`; `docker compose --profile robot config` with `docker-compose.gpu.yml` | no warnings; volumes, devices, environment and GPU reservation as documented. **Not built**: Docker Hub cannot be reached from the sandbox |

## 4. Not validated (needs the robot PC)

1. `./scripts/install.sh --robot`: the first build of `physios-robot:2.0.0`.
2. `./start_physios.sh --with-robot` with the real arm:
   * robot types on Home, including configs in `data/robot-config/`;
   * recording and camera streams;
   * `/task/status`;
   * ROS discovery with the arm's driver (same ROS domain, one Zenoh router).
3. Real `lerobot-train`, real model load; host rollout and container inference with the new
   image.
4. `scripts/import-legacy-data.py ~/lerobot/physical_ai_tools` on the real 1.x data (run with
   `--dry-run` first).
5. Dataset health probe; Hugging Face and GitHub publishing with real accounts.

## 5. Screenshots

`documentation/screenshots/` (light mode) and `documentation/screenshots/dark/` (dark mode)
hold every page, tab and view of 2.0.0 at 1600 × 900, taken against the devstack on 14
November 2026. `documentation/media/` holds the README banner and a screen recording of the
workflow (`rawbot-studio-demo.mp4`, with a GIF preview).
