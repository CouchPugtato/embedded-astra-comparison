#!/usr/bin/env bash
set -Eeo pipefail

if (( $# != 1 )); then
  echo 'Usage: bash scripts/replay_best.sh MODEL_NAME' >&2
  exit 2
fi
model_name=$1
if [[ ! "$model_name" =~ ^[[:alnum:]_.-]+$ ]] \
    || [[ "$model_name" == '.' ]] || [[ "$model_name" == '..' ]]; then
  echo 'Model name may contain only letters, numbers, periods, underscores, and hyphens.' >&2
  exit 2
fi

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
stats="results/$model_name/stats.csv"
if [[ ! -f "$stats" ]]; then
  echo "No results found for $model_name: $stats is missing." >&2
  exit 1
fi

selection="$(python3 - "$stats" <<'PY'
import csv
from pathlib import Path
import sys

with Path(sys.argv[1]).open(newline='', encoding='utf-8') as stream:
    rows = list(csv.DictReader(stream))
if not rows:
    raise SystemExit('stats.csv contains no attempts')
try:
    best = min(
        rows,
        key=lambda row: (
            -(int(row['targets_placed']) - int(row['distractors_in_bowl'])),
            float(row['runtime_seconds']),
            int(row['attempt']),
        ),
    )
except (KeyError, TypeError, ValueError) as error:
    raise SystemExit(f'invalid stats.csv: {error}') from error
score = int(best['targets_placed']) - int(best['distractors_in_bowl'])
print(
    f"{int(best['attempt']):03d} {score} "
    f"{float(best['runtime_seconds']):.3f} {int(best['seed'])}"
)
PY
)"
read -r attempt score runtime seed <<<"$selection"
policy="results/$model_name/attempt$attempt/policy.py"
if [[ ! -s "$policy" ]]; then
  echo "Selected policy is missing or empty: $policy" >&2
  exit 1
fi
echo "Replaying $model_name attempt $attempt (score $score, ${runtime}s)."

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
source /opt/ros/jazzy/setup.bash
set -u
timeout --foreground 600 colcon build --symlink-install
set +u
source install/setup.bash
set -u

simulator_pid=''
simulator_log="$(mktemp /tmp/tabletop_replay.XXXXXX.log)"
cleanup() {
  set +e
  if [[ -n "$simulator_pid" ]] && kill -0 -- "-$simulator_pid" 2>/dev/null; then
    kill -INT -- "-$simulator_pid" 2>/dev/null
    for _ in $(seq 1 10); do
      kill -0 -- "-$simulator_pid" 2>/dev/null || break
      sleep 1
    done
    if kill -0 -- "-$simulator_pid" 2>/dev/null; then
      kill -TERM -- "-$simulator_pid" 2>/dev/null
      sleep 3
    fi
    if kill -0 -- "-$simulator_pid" 2>/dev/null; then
      kill -KILL -- "-$simulator_pid" 2>/dev/null
    fi
    wait "$simulator_pid" 2>/dev/null
  fi
  rm -f -- "$simulator_log"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if pgrep -f '[r]os2 launch tabletop_sim tabletop_sim.launch.py|[g]z sim' >/dev/null; then
  echo 'A tabletop simulator is already running. Stop it before replaying.' >&2
  exit 1
fi

setsid ros2 launch tabletop_sim tabletop_sim.launch.py seed:="$seed" \
  >"$simulator_log" 2>&1 &
simulator_pid=$!

ready=0
for readiness_attempt in $(seq 1 90); do
  if ! kill -0 "$simulator_pid" 2>/dev/null; then
    echo 'Simulator exited during startup:' >&2
    tail -n 80 "$simulator_log" >&2
    exit 1
  fi
  if timeout --foreground 5 ros2 service list 2>/dev/null \
      | grep -qx '/controller_manager/list_controllers' \
    && timeout --foreground 5 ros2 action list 2>/dev/null | grep -qx '/move_action' \
    && timeout --foreground 5 ros2 action list 2>/dev/null \
      | grep -qx '/panda_hand_controller/gripper_cmd' \
    && timeout --foreground 5 ros2 topic list 2>/dev/null | grep -qx '/camera/image_raw'; then
    ready=1
    break
  fi
  sleep 2
done
if [[ "$ready" != 1 ]]; then
  echo 'Simulator did not become ready within 180 seconds.' >&2
  tail -n 80 "$simulator_log" >&2
  exit 1
fi

timeout --foreground 180 ros2 run tabletop_sim replay_policy \
  --policy-file "$policy" --seed "$seed" --max-runtime 120
