# Arena navigation — turn logic

Base navigation logic for a Raspberry Pi car that sweeps a closed, known-size
**rectangular** arena using 5 ultrasonic sensors (3 front, 2 right). This first
cut implements the **turn decision** only: it detects a wall straight ahead and
emits the turn to start the next lane. The right-side sensors are used purely to
keep the car parallel to and a fixed distance from the right wall. Full route
planning is stubbed (a serpentine left/right alternation).

There are no obstacles inside the arena, so the only thing the car ever turns
for is a wall straight ahead — the right side never triggers a turn on its own.

## Files

| File | Purpose |
|------|---------|
| `config.py` | **All** hardcoded values: arena size, sensor pins, distance thresholds, speeds, turn timing. Tune here, never in the logic. |
| `navigation.py` | Pure decision logic. `NavigationController.decide(readings) -> Command`. No hardware. |
| `sensors.py` | HC-SR04 driver. Reads real sensors on a Pi; returns `inf` (or a supplied simulator) elsewhere. |
| `motors.py` | Tank / skid-steer driver, one DIR + one PWM pin per motor. Prints intent in dry mode off-Pi. |
| `main.py` | The control loop: read → decide → drive. |
| `simulate.py` | Scripted off-hardware run that walks the car down a lane and into a front wall. |
| `test_navigation.py` | Assertions for every branch of the turn logic. |

## How decisions are made

`decide()` reduces the 5 readings to a couple of flags and picks one action:

1. **Wall straight ahead** (nearest of the 3 front sensors ≤ `FRONT_STOP_DISTANCE_CM`)
   → end of lane. `main.execute()` runs a **U-turn into the next lane**: rotate
   90°, drive forward `LANE_WIDTH_CM` (the sideways shift), rotate 90° the same
   way. Direction alternates every turn, **starting left**: left, right, left, …
   (independent of the side sensors). This is what marches the serpentine across
   the arena. The forward shift is open-loop timed as
   `LANE_WIDTH_CM / DRIVE_CM_PER_S`, so both must be tuned on the real car.
2. **Otherwise** → cruise forward, with a steering trim that holds the car
   parallel to and `RIGHT_TARGET_DISTANCE_CM` from the right wall. Slows to
   `SLOW_SPEED` inside `FRONT_SLOW_DISTANCE_CM`.

Missing / out-of-range sensors are passed as `float('inf')`.

## Run

```bash
python3 test_navigation.py   # unit tests, no hardware needed
python3 simulate.py          # scripted scenario, prints each decision
python3 sensor_test.py       # just read & print all sensors (wiring check)
python3 sensor_diagnostics.py # per-sensor pins + raw samples + distance (debug)
python3 main.py              # run on the car (mode set by config.USE_SENSORS)
```

## Bring-up: drive test first, sensors later

`config.USE_SENSORS` selects what `main.py` does on the car:

- **`USE_SENSORS = False`** (default) — open-loop **drive test**. Runs the
  hardcoded `DRIVE_TEST_SEQUENCE` (forward / left / right / stop, each for a set
  number of seconds) and never touches the sensors. Use this first to confirm
  the motors, H-bridge wiring and turning are correct. No sensors or ECHO
  voltage dividers need to be connected yet. Tune `TURN_TIME_S` here until the
  `left`/`right` steps give a clean 90°.
- **`USE_SENSORS = True`** — full sensor-driven navigation (read → decide →
  drive). Flip this on once the drive test looks right.

Edit `DRIVE_TEST_SEQUENCE` in `config.py` to script whatever maneuver you want
to check (e.g. a square: forward, right, forward, right, ...).

## Wiring it to real hardware

- Set the BCM pin numbers in `config.SENSORS` to match your wiring. Keep them
  clear of the motor pins (`12, 13, 16, 20`). Set a sensor's `enabled: False` to
  switch it off individually (its pins are left untouched and it always reads as
  "no echo") — handy for bringing sensors up one at a time. `USE_SENSORS` is the
  separate global switch between drive-test and navigation modes.
- Motors are DIR + PWM, one per side, in `config.MOTORS`: `dir` is the
  direction line (forward = HIGH), `pwm` is the speed line (0..100% duty). If a
  wheel spins the wrong way, set that side's `invert: True` instead of swapping
  wires.
- Turns are **open-loop timed**: the car spins at `TURN_SPEED` for `TURN_TIME_S`
  seconds (no IMU/encoders). Measure the real 90° turn time on the car and set
  `TURN_TIME_S` in `config.py`.

## Next steps (not yet implemented)

- **Lane counting / "done" condition.** The U-turn shifts one lane, but nothing
  counts lanes, so the car never knows when the whole arena is covered (≈
  `ARENA_WIDTH_CM / LANE_WIDTH_CM` lanes) and never stops on its own.
- Closed-loop turns and lane shift via odometry/IMU (everything is open-loop
  timed today: `TURN_TIME_S` and `LANE_WIDTH_CM / DRIVE_CM_PER_S`).
- Left-side sensors (the spec mentions possibly adding 2 per side) — add them to
  `config.SENSORS` and use them in `navigation.py`.
