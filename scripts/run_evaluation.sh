#!/usr/bin/env bash
set -Eeo pipefail

mode=full
model_name=model
while (( $# > 0 )); do
  case "$1" in
    --check)
      mode=check
      shift
      ;;
    --model)
      if (( $# < 2 )); then
        echo '--model requires a name' >&2
        exit 2
      fi
      model_name=$2
      shift 2
      ;;
    *)
      echo 'Usage: bash scripts/run_evaluation.sh [--check] [--model NAME]' >&2
      exit 2
      ;;
  esac
done
if [[ ! "$model_name" =~ ^[[:alnum:]_.-]+$ ]] \
    || [[ "$model_name" == '.' ]] || [[ "$model_name" == '..' ]]; then
  echo 'Model name may contain only letters, numbers, periods, underscores, and hyphens.' >&2
  exit 2
fi

progress() {
  printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

evaluation_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$evaluation_root"
public_root="$evaluation_root/results/$model_name"
private_root="$evaluation_root/private/$model_name"
if [[ "$mode" == full ]] && { [[ -e "$public_root" ]] || [[ -e "$private_root" ]]; }; then
  echo "Results already exist for $model_name. Choose a new --model name." >&2
  exit 2
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
source /opt/ros/jazzy/setup.bash
set -u

progress 'Building ROS packages...'
timeout --foreground 600 colcon build --symlink-install
set +u
source install/setup.bash
set -u

evaluation_seed=1

simulator_pid=''
simulator_log="$(mktemp /tmp/tabletop_sim.XXXXXX.log)"
cleanup() {
  set +e
  if [[ -n "$simulator_pid" ]] && kill -0 -- "-$simulator_pid" 2>/dev/null; then
    kill -INT -- "-$simulator_pid" 2>/dev/null
    for _ in $(seq 1 20); do
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

if pgrep -f '[r]os2 launch tabletop_sim tabletop_sim.launch.py' >/dev/null; then
  echo 'A tabletop_sim launch is already running. Stop it before using this managed run.' >&2
  exit 1
fi
if pgrep -f '[g]z sim' >/dev/null; then
  echo 'A Gazebo Sim process is already running. Stop it before using this managed run.' >&2
  exit 1
fi

progress 'Starting Gazebo, controllers, camera, and MoveIt...'
setsid ros2 launch tabletop_sim tabletop_sim.launch.py seed:="$evaluation_seed" \
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
  if (( readiness_attempt % 5 == 0 )); then
    progress "Still waiting for simulator readiness ($((readiness_attempt * 2))s)..."
  fi
  sleep 2
done
if [[ "$ready" != 1 ]]; then
  echo 'Simulator did not become ready within 180 seconds.' >&2
  tail -n 80 "$simulator_log" >&2
  exit 1
fi

progress 'Simulator ready. Preparing and capturing the scene...'
timeout --foreground 180 ros2 run tabletop_sim prepare_scene \
  --seed "$evaluation_seed" --timeout 30 --output prompts/initial.png

if [[ "$mode" == check ]]; then
  echo 'Quick check passed.'
  exit 0
fi

mkdir -p "$public_root" "$private_root"

run_attempt_managed() {
  local policy_path=$1
  local attempt=$2
  local seed=$3
  local private_path=$4
  local stats_path=$5
  local runtime=$6
  local workspace padded public_json command_status
  workspace="$(dirname -- "$policy_path")"
  printf -v padded '%03d' "$attempt"
  public_json="$workspace/attempt$padded/public_grade.json"

  progress "Running attempt $attempt of 25 from $policy_path..."
  set +e
  timeout --foreground 420 ros2 run tabletop_sim run_attempt \
    --attempt "$attempt" --seed "$seed" \
    --policy-file "$policy_path" --private-root "$private_path" \
    --stats "$stats_path" --max-runtime "$runtime"
  command_status=$?
  set -e
  progress "Attempt $attempt exited with status $command_status."

  if [[ ! -f "$public_json" ]]; then
    echo "Attempt exited $command_status without producing $public_json" >&2
    return 1
  fi
  python3 - "$public_json" <<'PY'
import json
from pathlib import Path
import sys

result = json.loads(Path(sys.argv[1]).read_text())
if result.get('policy_status') in {'reset_failed', 'grade_failed'}:
    raise SystemExit(f"infrastructure failure: {result.get('error')}")
PY
}

model_root="$public_root"
model_private="$private_root"
cp prompts/system.txt prompts/task.txt prompts/initial.png "$model_root/"

echo
echo "Beginning 25 attempts for $model_name."
echo "MODEL INPUT: $model_name attempt 1"
echo "Run the model with this as its entire workspace: $model_root"
echo 'Have it read system.txt, task.txt, and initial.png and create policy.py.'
while [[ ! -s "$model_root/policy.py" ]]; do
  read -r -p 'Press Enter after policy.py has been created... ' _
done

for attempt in $(seq 1 25); do
  if (( attempt > 1 )); then
    previous=$((attempt - 1))
    printf -v previous_padded '%03d' "$previous"
    echo
    echo "MODEL FEEDBACK: $model_name attempt $attempt"
    echo 'Have the same model review the previous images, action log, and public grade in:'
    echo "$model_root/attempt$previous_padded"
    echo 'Then have it revise policy.py for the next attempt.'
    read -r -p 'Press Enter after policy.py is ready... ' _
    test -s "$model_root/policy.py"
  fi
  run_attempt_managed "$model_root/policy.py" "$attempt" "$evaluation_seed" \
    "$model_private" "$model_root/stats.csv" 120
done

python3 - "$model_root" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
candidates = []
for path in sorted(root.glob('attempt[0-9][0-9][0-9]/public_grade.json')):
    grade = json.loads(path.read_text(encoding='utf-8'))
    criteria = grade['criteria']
    bowl_stable = bool(criteria.get('bowl_stable'))
    targets = criteria['repeated_color_cubes_in_bowl']['placed']
    distractors = criteria['unique_color_cubes_in_bowl']
    rank = (
        bool(grade.get('task_success')),
        bowl_stable,
        targets,
        -distractors,
        grade.get('policy_status') == 'completed',
        -float(grade.get('runtime_seconds', 0.0)),
    )
    candidates.append((rank, path.parent.name, grade))

_, directory, grade = max(candidates)
result = {
    'attempt': int(directory.removeprefix('attempt')),
    'artifact_directory': directory,
    'public_grade': grade,
}
(root / 'best_attempt.json').write_text(
    json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
print(f'Best result: {root / "best_attempt.json"}')
PY

echo "Evaluation complete. Public results: $public_root"
echo "Privileged grades: $private_root"
