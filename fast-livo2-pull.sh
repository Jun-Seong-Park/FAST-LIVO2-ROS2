#!/usr/bin/env bash
# Pull ~/FAST-LIVO2-ROS2 from GitHub.
# Local changes are temporarily stashed via autostash, then reapplied on top of the rebase.
# Usage:
#   ~/.claude/fast-livo2-pull.sh

set -e
cd "$HOME/FAST-LIVO2-ROS2"

git pull --rebase --autostash
