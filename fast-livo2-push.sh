#!/usr/bin/env bash
# Push local changes in ~/FAST-LIVO2-ROS2 to GitHub.
# Usage:
#   ~/.claude/fast-livo2-push.sh                # message = current time
#   ~/.claude/fast-livo2-push.sh "message text" # specify the message directly

set -e
cd "$HOME/FAST-LIVO2-ROS2"

MSG="${1:-push $(date '+%Y-%m-%d %H:%M:%S')}"

git add .

if git diff --cached --quiet; then
    echo "No changes — skipping commit"
else
    git commit -m "$MSG"
fi

git push
