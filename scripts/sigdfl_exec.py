#!/usr/bin/env python3
"""
Reset SIGINT, SIGTERM and SIGPIPE to their default dispositions, then exec the given command.

A non-interactive bash starts background jobs with SIGINT ignored, and bash cannot reset a signal
that was ignored on entry. `ros2 launch` started that way never reacts to SIGINT, so it cannot run
its orderly shutdown (forward SIGINT, then escalate to SIGTERM and SIGKILL on its own timeouts).

Usage: sigdfl_exec.py <command> [args...]
"""
import os
import signal
import sys

signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
signal.signal(signal.SIGPIPE, signal.SIG_DFL)   # Python ignores SIGPIPE; do not pass that on
os.execvp(sys.argv[1], sys.argv[1:])
