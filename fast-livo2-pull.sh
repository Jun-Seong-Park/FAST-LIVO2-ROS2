#!/usr/bin/env bash
# ~/FAST-LIVO2-ROS2 를 GitHub 에서 pull.
# 로컬 변경사항은 autostash 로 임시 보관 후 rebase 위에 다시 적용.
# 사용법:
#   ~/.claude/fast-livo2-pull.sh

set -e
cd "$HOME/FAST-LIVO2-ROS2"

git pull --rebase --autostash
