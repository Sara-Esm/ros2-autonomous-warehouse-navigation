#!/usr/bin/env bash
# Is SIGINT ignored for a process started in the background by a non-interactive bash script?
# Compares a plain background launch with the SIG_DFL exec wrapper. Reads /proc/<pid>/status SigIgn.
WRAP="$(dirname "$0")/sigdfl_exec.py"

report() {
  local label=$1 pid=$2 mask
  mask=$(awk '/SigIgn/{print $2}' "/proc/$pid/status")
  echo "$label: SigIgn=$mask SIGINT_ignored=$(( (16#$mask >> 1) & 1 ))"
}

setsid sleep 30 &
P1=$!
setsid python3 "$WRAP" sleep 30 &
P2=$!
sleep 1
report "plain background setsid" "$P1"
report "background via sigdfl_exec.py" "$P2"
kill "$P1" "$P2"
