#!/usr/bin/env bash
# Run a project script with its output mirrored to the systemd journal, where
# Filebeat ships it to Kibana (Discover filter: log.syslog.appname:"roomba-bot").
# The terminal still shows everything live, exactly like running python directly.
#
# Activate the venv first, then:
#   ./run.sh main.py [args...]
set -euo pipefail
[ $# -ge 1 ] || { echo "usage: $0 <script.py> [args...]" >&2; exit 2; }

tag=roomba-bot
run_id="$(date +%Y%m%d-%H%M%S)-$$"

echo "=== run $run_id start: $* ===" | systemd-cat -t "$tag"
status=0
# -u: unbuffered, so lines reach the journal live and nothing is lost on a crash.
if [ -t 1 ]; then
    python3 -u "$@" 2>&1 | tee /dev/tty | systemd-cat -t "$tag" || status=$?
else
    python3 -u "$@" 2>&1 | systemd-cat -t "$tag" || status=$?
fi
echo "=== run $run_id end: $* (exit $status) ===" | systemd-cat -t "$tag"
exit $status
