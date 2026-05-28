#!/usr/bin/env bash
# ~/FAST-LIVO2-ROS2 의 로컬 변경사항을 GitHub 로 push.
# 사용법:
#   ~/.claude/fast-livo2-push.sh                # 메시지 = 현재 시각
#   ~/.claude/fast-livo2-push.sh "메시지 내용"  # 메시지 직접 지정

set -e
cd "$HOME/FAST-LIVO2-ROS2"

MSG="${1:-push $(date '+%Y-%m-%d %H:%M:%S')}"

git add .

if git diff --cached --quiet; then
    echo "변경사항 없음 — commit 생략"
else
    git commit -m "$MSG"
fi

git push
