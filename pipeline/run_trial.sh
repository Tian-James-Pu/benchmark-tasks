#!/usr/bin/env bash
# Run one trial of a task on a free local GPU, render its videos, and print the score.
#
#   pipeline/run_trial.sh <task-folder> demo  [max_steps]   scripted demo (yam_scripted), 640x360 frames
#   pipeline/run_trial.sh <task-folder> dream [max_steps]   Dream-1, 320x176 frames (spends Dream credits)
#
# TASK_FILE=redteam_drag.py runs another file in the folder instead of task.py (a red-team variant of the demo);
# its video goes to media/<mode>-<file>.mp4 and it never replaces setup.png.
#
# The instruction comes from tasks/<task-folder>/task.json, or $INSTRUCTION when there is none yet.
# Output: ~/yam-eval/<task-folder>/<policy>-<time>/ (log, frames, videos) and, in
# ~/yam-eval/<task-folder>/media/, <policy>.mp4 (H.264, top camera) and setup.png (first top-camera frame).
# Authorized physical GPUs are listed in pipeline/gpu-allow (default "2 3"). GPUs 0 and 1 belong to
# labmates and are used only while the user has temporarily allowed them in that file.
# Trials take a slot on an authorized GPU (GPU_SLOTS per card, default 3).
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TASK=${1:?task folder name under tasks/}
MODE=${2:?demo or dream}
DIR=$REPO/tasks/$TASK
FILE=${TASK_FILE:-task.py}
[[ -f $DIR/$FILE ]] || { echo "run_trial: $DIR/$FILE not found" >&2; exit 2; }
case $MODE in
demo) POLICY=yam_scripted; STEPS=${3:-900}; W=640; H=360 ;;
dream) POLICY=dream; STEPS=${3:-2400}; W=320; H=176 ;;
*) echo "run_trial: mode must be demo or dream" >&2; exit 2 ;;
esac
if [[ -z ${INSTRUCTION:-} ]]; then
  [[ -f $DIR/task.json ]] || { echo "run_trial: no task.json; set INSTRUCTION" >&2; exit 2; }
  INSTRUCTION=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["instruction"])' "$DIR/task.json")
fi

# shellcheck disable=SC1090
source "$HOME/IsaacLab-Arena/yam.env"
LOCKS=${TMPDIR:-/tmp}/yam-gpu-locks
mkdir -p "$LOCKS"
# GPU_SLOTS trials may share one GPU (default 3; a trial uses a few GB and about half a card).
SLOTS=${GPU_SLOTS:-3}
GPU=
while [[ -z $GPU ]]; do
  # Fill the first slot on every GPU before a second one. Prefer lower current
  # utilization, then lower memory use; unrelated jobs can saturate a card while
  # using less memory than an idle cached model on another card.
  for s in $(seq 1 "$SLOTS"); do
    for g in $(nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader,nounits | sort -t, -k3,3n -k2,2n | cut -d, -f1); do
      ALLOW=$(cat "$REPO/pipeline/gpu-allow" 2>/dev/null || echo "2 3")
      case " $ALLOW " in *" $g "*) ;; *) continue ;; esac
      exec {FD}>"$LOCKS/gpu$g.slot$s.lock"
      if flock -n "$FD"; then GPU=$g; break 2; fi
      exec {FD}>&-
    done
  done
  [[ -n $GPU ]] || sleep 5
done

# Mask by physical UUID so CUDA initialization cannot allocate on reserved cards.
# Inside this process the selected physical GPU is logical cuda:0. The installed
# AppLauncher also selects the renderer with the masked activeCudaGpus index.
GPU_UUID=$(nvidia-smi -i "$GPU" --query-gpu=uuid --format=csv,noheader)
[[ $GPU_UUID == GPU-* ]] || { echo "run_trial: cannot resolve GPU $GPU UUID" >&2; exit 2; }
export CUDA_VISIBLE_DEVICES="$GPU_UUID"

OUT=$HOME/yam-eval/$TASK/$MODE-$(date +%m%d-%H%M%S)-gpu$GPU-$$
MEDIA=$HOME/yam-eval/$TASK/media
mkdir -p "$OUT" "$MEDIA"
echo "run_trial: $TASK $MODE on GPU $GPU, up to $STEPS steps -> $OUT"
cd "$REPO"
set +e
inspect-robots run --embodiment yam_arena --policy "$POLICY" \
  -E task="tasks/$TASK/$FILE" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=$W -E cam_height=$H -E device="cuda:0" \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps "$STEPS" \
  --store-frames --log-dir "$OUT" >"$OUT/run.log" 2>&1
RC=$?
set -e
grep -E "^run status|^outcome|^error|task_score" "$OUT/run.log" || true
LOG=$(ls "$OUT"/*.json 2>/dev/null | grep -v live | head -1 || true)
if [[ $RC -ne 0 || -z $LOG ]] || grep -q "^run status: error" "$OUT/run.log"; then
  echo "run_trial: FAILED (see $OUT/run.log)"; exit 1
fi
inspect-robots video "$LOG" >/dev/null 2>&1
TOP=$(find "$OUT/frames" -name '*top_cam.mp4' | head -1)
NAME=$MODE; [[ $FILE == task.py ]] || NAME=$MODE-${FILE%.py}
ffmpeg -y -loglevel error -i "$TOP" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$MEDIA/$NAME.mp4"
python - "$OUT" "$MEDIA/setup.png" "$NAME" <<'PY'
import glob, sys
import numpy as np
from PIL import Image
out, png, mode = sys.argv[1:]
frames = sorted(glob.glob(f"{out}/frames/*/*top_cam_*.npy"))
print(f"run_trial: {len(frames)} steps recorded")
if mode == "demo":  # the screenshot comes from the higher-resolution demo run
    Image.fromarray(np.load(frames[0])).save(png)
# A contact sheet of six frames, to look at the run without playing the video.
idx = [round(i * (len(frames) - 1) / 5) for i in range(6)]
tiles = [np.load(frames[i]) for i in idx]
Image.fromarray(np.concatenate([np.concatenate(tiles[:3], 1), np.concatenate(tiles[3:], 1)], 0)).save(png.replace("setup.png", f"{mode}-sheet.png"))
print("run_trial: contact sheet", png.replace("setup.png", f"{mode}-sheet.png"))
PY
echo "run_trial: video $MEDIA/$NAME.mp4"
# The raw frames (hundreds of MB per run) have served their purpose: keep the videos and the log.
find "$OUT/frames" -name '*.npy' -delete
# Dream-1 credits are limited: keep a ledger of every Dream run.
[[ $MODE == dream ]] && printf '%s\t%s\t%s\t%s\n' "$(date -u +%FT%TZ)" "$TASK" "$(grep -oE 'task_score: [0-9.]+' "$OUT/run.log" | tail -1)" "$OUT" >> "$REPO/pipeline/dream_runs.tsv"
true
