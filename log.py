"""Structured logging: one compact JSON line per event.

When runs are shipped to Kibana (run.sh -> journald -> Filebeat), lines that
start with {"event" are decoded into roomba.* fields, so values like sensor
distances, actions, and duty cycles can be filtered and charted, not just
text-searched.
"""
import json
import math


def log(event, **fields):
    rec = {"event": event}
    for k, v in fields.items():
        # Strict JSON has no Infinity/NaN (out-of-range sensor reads are inf).
        if isinstance(v, float) and not math.isfinite(v):
            v = None
        rec[k] = v
    print(json.dumps(rec, default=str), flush=True)
