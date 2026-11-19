#!/usr/bin/env bash
# Copy a task's demo video and frame sheet (and its Dream-1 video if any) into pipeline/artifact/media/, ready to
# upload to the tracker page as assets.
#   pipeline/tracker_media.sh <task-folder>
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TASK=${1:?task folder}; M=$HOME/yam-eval/$TASK/media; OUT=$REPO/pipeline/artifact/media
mkdir -p "$OUT"
cp "$M/demo.mp4" "$OUT/$TASK-demo.mp4"
[[ -s $M/dream.mp4 ]] && cp "$M/dream.mp4" "$OUT/$TASK-dream.mp4"
source "$HOME/IsaacLab-Arena/yam.env"
python -c "from PIL import Image; import sys; Image.open(sys.argv[1]).convert('RGB').save(sys.argv[2], quality=82)" \
  "$M/demo-sheet.png" "$OUT/$TASK-sheet.jpg"
ls "$OUT"/"$TASK"-*
