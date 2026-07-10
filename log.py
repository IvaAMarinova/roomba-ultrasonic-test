"""Structured logging: one compact JSON line per event.

When runs are shipped to Kibana (run.sh -> journald -> Filebeat), lines that
start with {"event" are decoded into roomba.* fields, so values like sensor
distances, actions, and duty cycles can be filtered and charted, not just
text-searched.
"""
import json
import math
from dataclasses import dataclass
from itertools import cycle


@dataclass
class Logger:
    format: str

    def log(self, event: str, **fields):
        rec = {"event": event}
        for k, v in fields.items():
            # Strict JSON has no Infinity/NaN (out-of-range sensor reads are inf).
            if isinstance(v, float) and not math.isfinite(v):
                v = None
            rec[k] = v
        if self.format == 'json':
            print(json.dumps(rec, default=str), flush=True)
        else:
            _readable_print(rec)


_colors = {
    'black': '30',
    'red': '31',
    'green': '32',
    'yellow': '33',
    'blue': '34',
    'magenta': '35',
    'cyan': '36',
    'white': '37'
}


def _readable_print(rec: dict):
    kv_pairs: list[str] = []
    key_colors = cycle(color for color in _colors.keys() if color != 'black')
    for key, value in rec.items():
        if key == 'event':
            continue
        kv_pairs.append(_color(next(key_colors), key + ':', bold=True))
        if isinstance(value, float):
            kv_pairs.append(f'{value:.2f}')
        else:
            kv_pairs.append(value)
    print(_color("yellow", rec["event"], bold=True), *kv_pairs)


def _color(color: str, text: str, bold: bool = False) -> str:
    color_code = _colors.get(color.lower(), '37')
    style_code = '1' if bold else '0'

    return f"\x1b[{style_code};{color_code}m{text}\x1b[0m"
