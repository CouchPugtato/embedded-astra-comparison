#!/usr/bin/env bash
set -Eeo pipefail

mode=full
resume=0
model_name=model
while (( $# > 0 )); do
  case "$1" in
    --check)
      mode=check
      shift
      ;;
    --resume)
      resume=1
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
      echo 'Usage: bash scripts/run_evaluation.sh [--check] [--resume] [--model NAME]' >&2
      exit 2
      ;;
  esac
done
if [[ "$mode" == check && "$resume" == 1 ]]; then
  echo '--check and --resume cannot be used together.' >&2
  exit 2
fi
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
evaluation_seed=1
start_attempt=1
if [[ "$mode" == full && "$resume" == 1 ]]; then
  if [[ ! -d "$public_root" || ! -d "$private_root" ]]; then
    echo "Cannot resume $model_name: both $public_root and $private_root must exist." >&2
    exit 2
  fi
  start_attempt="$(python3 - "$public_root" "$private_root" "$evaluation_seed" <<'PY'
import csv
import json
from pathlib import Path
import sys

public_root = Path(sys.argv[1])
private_root = Path(sys.argv[2])
seed = int(sys.argv[3])

for name in ('system.txt', 'task.txt', 'initial.png', 'policy.py', 'stats.csv'):
    path = public_root / name
    if not path.is_file() or path.stat().st_size == 0:
        raise SystemExit(f'cannot resume: missing or empty {path}')

with (public_root / 'stats.csv').open(newline='', encoding='utf-8') as stream:
    rows = list(csv.DictReader(stream))
try:
    row_attempts = [int(row['attempt']) for row in rows]
except (KeyError, TypeError, ValueError) as error:
    raise SystemExit(f'cannot resume: invalid stats.csv: {error}') from error

public_attempts = sorted(
    int(path.name.removeprefix('attempt'))
    for path in public_root.glob('attempt[0-9][0-9][0-9]')
    if path.is_dir()
)
private_attempts = sorted(
    int(path.name.removeprefix('attempt'))
    for path in private_root.glob('attempt[0-9][0-9][0-9]')
    if path.is_dir()
)
expected = list(range(1, len(rows) + 1))
if row_attempts != expected:
    raise SystemExit(f'cannot resume: stats attempts must be consecutive: {row_attempts}')
if public_attempts != expected or private_attempts != expected:
    raise SystemExit(
        'cannot resume: public, private, and stats attempts do not match '
        f'(public={public_attempts}, private={private_attempts}, stats={expected})'
    )
if len(expected) > 25:
    raise SystemExit('cannot resume: evaluation contains more than 25 attempts')

for attempt in expected:
    directory = f'attempt{attempt:03d}'
    public_grade = public_root / directory / 'public_grade.json'
    private_grade = private_root / directory / 'private_grade.json'
    policy = public_root / directory / 'policy.py'
    for path in (public_grade, private_grade, policy):
        if not path.is_file() or path.stat().st_size == 0:
            raise SystemExit(f'cannot resume: missing or empty {path}')
    for path in (public_grade, private_grade):
        result = json.loads(path.read_text(encoding='utf-8'))
        if result.get('attempt') != attempt or result.get('seed') != seed:
            raise SystemExit(f'cannot resume: attempt or seed mismatch in {path}')
    if int(rows[attempt - 1]['seed']) != seed:
        raise SystemExit(f'cannot resume: seed mismatch in stats attempt {attempt}')

print(len(expected) + 1)
PY
)"
elif [[ "$mode" == full ]] && { [[ -e "$public_root" ]] || [[ -e "$private_root" ]]; }; then
  echo "Results already exist for $model_name. Use --resume or choose a new model name." >&2
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

if [[ "$resume" == 0 ]]; then
  mkdir -p "$public_root" "$private_root"
fi

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
if [[ "$resume" == 0 ]]; then
  cp prompts/system.txt prompts/task.txt prompts/initial.png "$model_root/"
  echo
  echo "Beginning 25 attempts for $model_name."
  echo "MODEL INPUT: $model_name attempt 1"
  echo "Run the model with this as its entire workspace: $model_root"
  echo 'Have it read system.txt, task.txt, and initial.png and create policy.py.'
  while [[ ! -s "$model_root/policy.py" ]]; do
    read -r -p 'Press Enter after policy.py has been created... ' _
  done
else
  echo
  if (( start_attempt <= 25 )); then
    echo "Resuming $model_name at attempt $start_attempt of 25."
  else
    echo "$model_name already has all 25 attempts; finalizing results."
  fi
fi

for ((attempt = start_attempt; attempt <= 25; attempt++)); do
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
