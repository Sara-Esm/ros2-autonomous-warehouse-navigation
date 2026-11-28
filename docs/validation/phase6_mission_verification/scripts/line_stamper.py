#!/usr/bin/env python3
"""Prefix every stdin line with the wall-clock receipt time (time.time(), same clock as `date +%s.%N`)."""
import sys, time
for line in sys.stdin:
    sys.stdout.write(f'{time.time():.3f} {line}'); sys.stdout.flush()
