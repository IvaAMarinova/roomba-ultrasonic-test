# Arena navigation — Roomba-style ultrasonic car

A Raspberry Pi car that sweeps a closed, known-size **rectangular** arena in a
serpentine ("boustrophedon") pattern using 5 ultrasonic sensors (3 facing front,
2 facing right) and an optional BNO086 IMU for accurate turns.

There are no obstacles inside the arena, so the only thing the car ever turns for
is a **wall straight ahead**. The right-side sensors are used purely to keep the
car driving parallel to, and a fixed distance from, the right wall.

**Everything you tune lives in [`config.py`](config.py).** You should never have
to edit the logic files to change behaviour — read this doc, open `config.py`,
change numbers. The rest of this README explains what each number does.

---

## 1. The big picture: how the car drives

The car runs one loop (`main.py`): **read sensors → decide → drive**, ~20×/sec.

```
        ┌─ wall ahead? (front sensors) ──── yes ──►  U-TURN into next lane
read ──►│                                            (spin 90°, shift sideways,
        └─ no ──►  CRUISE forward, trimming           spin 90° same way)
                   against the right wall
```

### Cruising (the normal state)

While no wall is close ahead, the car drives forward at `DRIVE_SPEED`. A gentle
**steering trim** nudges it to stay parallel to the right wall at
`RIGHT_TARGET_DISTANCE_CM`. If a wall starts getting close ahead (within
`FRONT_SLOW_DISTANCE_CM`) it drops to `SLOW_SPEED` to prepare for the turn.

### The U-turn (end of a lane)

When the front sensors agree a wall is within `FRONT_STOP_DISTANCE_CM`, the lane
is over and the car does a **U-turn into the next lane**, as one atomic maneuver:

1. **Spin 90°** in place.
2. **Drive forward `LANE_WIDTH_CM`** — this is the sideways shift into the next
   lane (the car is now pointed across the arena).
3. **Spin 90° the same direction** — now it's facing back down the new lane.

Turn direction **alternates every U-turn, starting LEFT**: left, right, left, …
That alternation is what snakes the car back and forth across the whole arena.

### Skid steer (how it physically moves)

It's a **tank / skid-steer** car: two sides (left, right), each with a direction
pin and a PWM speed pin. Forward = both sides forward. Turn in place = one side
forward, the other reverse. There is no steering servo. See `motors.py`.

---

## 2. How turning works (read this before tuning turns)

There are **two ways** to do the 90° spins, chosen by `USE_IMU_TURN`:

### A) IMU closed-loop turns (`USE_IMU_TURN = True`, recommended)

With a BNO086 IMU connected, the car spins and **watches its real heading**,
stopping once it has actually rotated `TURN_ANGLE_DEG` (minus a small
`IMU_TURN_TOLERANCE_DEG` to allow for coast). This stays accurate even as the
battery drains or the floor friction changes — it does not depend on hand-tuned
timing.

It accumulates the heading change sample-by-sample (wrap-safe around ±180°),
ignores corrupted I2C spikes (`IMU_GLITCH_MAX_STEP_DEG`), and has a hard timeout
(`IMU_TURN_TIMEOUT_S`) so a spin can never run forever.

**Stall recovery (tire-tension boost).** Spinning in place puts a lot of
sideways scrub on the tires; sometimes they bind and the car barely rotates at
`TURN_SPEED`. So: if after `IMU_TURN_BOOST_AFTER_S` seconds the IMU shows we've
turned **less than half** of `TURN_ANGLE_DEG`, the code **bumps the spin speed
up** (by `IMU_TURN_BOOST_FACTOR`, capped at full power) to break through the
friction, and holds that higher speed for the rest of the spin. You'll see a
`[u-turn] stall: ...` line in the log when this kicks in.

If the IMU is missing or fails to start, the car automatically falls back to
timed turns (below) — nothing breaks without the IMU.

### B) Timed open-loop turns (`USE_IMU_TURN = False`, or no IMU)

The car just spins at `TURN_SPEED` for `TURN_TIME_S` seconds and hopes that's
90°. Simple, no IMU needed, but you must **measure `TURN_TIME_S` on the real
car** and it drifts as the battery drains. Use this only for early bring-up or if
you have no IMU.

> The sideways lane shift (step 2 of the U-turn) is **always** open-loop timed as
> `LANE_WIDTH_CM / DRIVE_CM_PER_S` seconds, regardless of `USE_IMU_TURN`. Both of
> those numbers must be measured on the real car.

---

## 3. Two run modes: drive test vs. navigation

`USE_SENSORS` in `config.py` picks what `main.py` does on the car:

- **`USE_SENSORS = False` → open-loop drive test.** Runs the hardcoded
  `DRIVE_TEST_SEQUENCE` (a scripted list of forward / left / right / stop steps)
  and **never reads the ultrasonic sensors**. Use this *first* to confirm the
  motors, H-bridge wiring, and turning all work. No sensors or echo voltage
  dividers need to be connected yet. (The `left`/`right` steps still use the IMU
  if `USE_IMU_TURN = True`.)
- **`USE_SENSORS = True` → full sensor-driven navigation.** The real read →
  decide → drive loop. Flip this on once the drive test looks good.

---

## 4. Quickstart for the car

```bash
# 1. Bring-up: check motors + turning, no sensors needed.
#    In config.py set  USE_SENSORS = False
python3 main.py

# 2. (No IMU? Tune the timed turn.) With USE_IMU_TURN = False, adjust
#    TURN_TIME_S until the left/right steps give a clean 90°.

# 3. Go live: in config.py set  USE_SENSORS = True
python3 main.py
```

Off the car (on a laptop, no Raspberry Pi) everything still runs — motors print
their intent instead of moving, and sensors read "nothing". Useful tests:

```bash
python3 test_navigation.py     # unit tests for the decision logic
python3 simulate.py            # scripted scenario, prints each decision
python3 sensor_test.py         # read & print all sensors (wiring check, on Pi)
python3 sensor_diagnostics.py  # per-sensor pins + raw samples (debug, on Pi)
python3 imu_turn_test.py       # spin once using the IMU and report the result
```

Stop the car any time with **Ctrl-C** — it stops the motors and cleans up GPIO.

---

## 5. `config.py` — the complete tuning reference

This is the part to share with your friends. Everything is grouped the same way
as in the file.

### 5.1 Arena geometry

| Setting | What it does | When to change |
|---|---|---|
| `ARENA_WIDTH_CM` | Arena size across the lanes (the direction the car steps sideways). | Match your real arena. |
| `ARENA_LENGTH_CM` | Arena size along each lane (the long runs). | Match your real arena. |
| `ROBOT_WIDTH_CM` | Physical width of the car. | Measure your car. |
| `LANE_WIDTH_CM` | How far the car shifts sideways at each U-turn. **Must be ≤ `ROBOT_WIDTH_CM`** or it leaves unswept strips; a little less gives overlap. Bigger = faster but riskier coverage. | Tune for coverage vs. speed. |
| `DRIVE_CM_PER_S` | **Measured** forward speed at `DRIVE_SPEED`, in cm/s. Used to convert the lane shift into a drive time. | **Measure on the real car:** drive forward at `DRIVE_SPEED` for a known time, divide distance by time. |

> The sideways shift takes `LANE_WIDTH_CM / DRIVE_CM_PER_S` seconds. If the car
> shifts too far or too little after a U-turn, fix `DRIVE_CM_PER_S` first (it's
> probably wrong), then `LANE_WIDTH_CM`.

### 5.2 Sensors (`SENSORS` dict + groupings)

Each sensor is `name: {"trig": <pin>, "echo": <pin>, "enabled": <bool>}` in
**BCM** pin numbering.

- **`trig` / `echo`** — set these to match your actual wiring. Keep them clear of
  the motor pins (`12, 13, 16, 20`).
- **`enabled`** — `False` switches one sensor off: its pins are left untouched
  and it always reports "no echo". Great for bringing sensors up one at a time,
  or ignoring a broken one. (Note: in the default config the two `right_*`
  sensors are `enabled: False` — turn them on to use wall-following.)
- **`FRONT_SENSORS` / `RIGHT_SENSORS`** — which names count as "front" (used to
  detect the end-of-lane wall) and "right" (used for wall-following). Update
  these if you rename or re-layout sensors.

### 5.3 Decision thresholds (centimetres)

| Setting | What it does |
|---|---|
| `FRONT_STOP_DISTANCE_CM` | Wall this close ahead → end of lane, **do the U-turn**. |
| `FRONT_SLOW_DISTANCE_CM` | Start slowing down (`SLOW_SPEED`) when a wall is this close. **Must be larger than `FRONT_STOP_DISTANCE_CM`.** |
| `RIGHT_WALL_DISTANCE_CM` | Closer than this on the right ⇒ a wall is present there. |
| `RIGHT_TARGET_DISTANCE_CM` | Desired gap to the right wall while wall-following. |

The front wall is detected on the **median** of the 3 front sensors, so one bad
sensor can't trigger or block a turn on its own — at least two must agree.

### 5.4 Sensor reliability

| Setting | What it does |
|---|---|
| `SENSOR_MAX_RANGE_CM` / `SENSOR_MIN_RANGE_CM` | Readings outside this band are thrown out as junk. |
| `SENSOR_TIMEOUT_S` | Give up waiting for an echo after this long (no echo → "nothing there"). |
| `SENSOR_SAMPLES` | Median-of-N pings per reading to reject noise. Higher = steadier but slower. |
| `SOUND_SPEED_CM_PER_S` | Speed of sound used to convert echo time to distance. Rarely changed. |

### 5.5 Motion parameters

| Setting | What it does | Range |
|---|---|---|
| `DRIVE_SPEED` | Normal forward speed. | 0..1 |
| `SLOW_SPEED` | Forward speed when a wall is getting close. | 0..1 |
| `TURN_SPEED` | In-place rotation speed (the base speed for both IMU and timed spins). | 0..1 |
| `STEER_CORRECTION_GAIN` | How hard the wall-follow trims heading. Too high = wobble; too low = drifts into/away from the wall. | small |
| `MAX_STEER_TRIM` | Clamp on the wall-follow trim so it never oversteers. | 0..1 |
| `TURN_TIME_S` | Seconds to spin 90° at `TURN_SPEED`. **Only used as the fallback when there's no IMU.** Measure it on the car. | seconds |

### 5.6 IMU turning (closed-loop) — the important turn knobs

| Setting | What it does |
|---|---|
| `USE_IMU_TURN` | **Master switch.** `True` = turn by measured IMU heading (auto-falls back to timed if the IMU is missing). `False` = always use timed `TURN_TIME_S`. |
| `TURN_ANGLE_DEG` | Target rotation for one spin (90° for a normal U-turn). |
| `IMU_TURN_TOLERANCE_DEG` | Stop this many degrees early to allow for momentum/coast. Raise it if the car consistently overshoots. |
| `IMU_TURN_TIMEOUT_S` | Safety cap: a spin never runs longer than this, even if it never reaches the target. |
| `IMU_GLITCH_MAX_STEP_DEG` | Per-sample heading jumps bigger than this are treated as corrupted I2C reads and ignored. |
| `IMU_TURN_POLL_S` | How often to re-read the heading during a spin (smaller = more responsive, more I2C traffic). |
| `IMU_TURN_BOOST_AFTER_S` | **Stall recovery.** If the spin has run this long and the IMU shows less than **half** of `TURN_ANGLE_DEG`, the tires are probably binding → speed up. Set to `None` to disable the boost. |
| `IMU_TURN_BOOST_FACTOR` | How much to multiply `TURN_SPEED` by when a stall is detected (capped at full power, 1.0). E.g. `0.7 × 1.4 ≈ 0.98`. |

**Tuning the stall boost:** if your car often grinds and barely turns, *lower*
`IMU_TURN_BOOST_AFTER_S` (boost sooner) and/or *raise* `IMU_TURN_BOOST_FACTOR`
(boost harder). If a normal 90° turn already finishes in well under
`IMU_TURN_BOOST_AFTER_S`, the boost will never trigger — lower the threshold so
it can actually help before `IMU_TURN_TIMEOUT_S` ends the spin.

### 5.7 Motors (`MOTORS` dict)

Each side is `name: {"dir": <pin>, "pwm": <pin>, "invert": <bool>}` in BCM.

- **`dir`** — direction line (forward = HIGH). **`pwm`** — speed line (0..100% duty).
- **`invert`** — if a wheel spins the **wrong way**, flip `invert` for that side
  instead of re-wiring it.
- `MOTOR_PWM_HZ` — PWM frequency on the speed pins.
- `MOTOR_DEADZONE` — any side command below this magnitude counts as "stop"
  (avoids buzzing motors at tiny duty cycles).

### 5.8 Control loop

| Setting | What it does |
|---|---|
| `CONTROL_LOOP_HZ` | How often the car reads sensors and decides (≈20 Hz). |

### 5.9 Bring-up & drive test

| Setting | What it does |
|---|---|
| `USE_SENSORS` | `False` = open-loop drive test (ignore sensors). `True` = full navigation. |
| `DRIVE_TEST_SEQUENCE` | The scripted maneuver for drive-test mode: a list of `(action, seconds)` where action is `"forward"`, `"left"`, `"right"`, or `"stop"`. Edit this to script any check you want (e.g. drive a square). `left`/`right` steps turn ~90° (via IMU if enabled). |

---

## 6. Common "I want to…" recipes

| You want to… | Change in `config.py` |
|---|---|
| Drive faster / slower | `DRIVE_SPEED` (and re-measure `DRIVE_CM_PER_S`). |
| Turn faster | `TURN_SPEED` (and re-measure `TURN_TIME_S` if not using the IMU). |
| Fix turns that overshoot 90° | Raise `IMU_TURN_TOLERANCE_DEG` (IMU) or lower `TURN_TIME_S` (timed). |
| Fix turns that undershoot / grind | Lower `IMU_TURN_BOOST_AFTER_S` and/or raise `IMU_TURN_BOOST_FACTOR`. |
| Sweep more thoroughly (no gaps) | Lower `LANE_WIDTH_CM` (more overlap, slower). |
| Turn earlier/later at the end wall | `FRONT_STOP_DISTANCE_CM`. |
| Hug the right wall closer/looser | `RIGHT_TARGET_DISTANCE_CM`. |
| Car wobbles while wall-following | Lower `STEER_CORRECTION_GAIN`. |
| A wheel spins backwards | Flip `invert` for that side in `MOTORS`. |
| Bring sensors up one at a time | Set `enabled: False` on the ones you're not testing. |
| Run without an IMU | `USE_IMU_TURN = False`, then tune `TURN_TIME_S`. |
| Just test motors/turning | `USE_SENSORS = False`, edit `DRIVE_TEST_SEQUENCE`. |

---

## 7. Files

| File | Purpose |
|---|---|
| `config.py` | **All** tunable values. Change behaviour here, never in the logic. |
| `main.py` | The control loop and the U-turn / spin (incl. IMU + stall boost) logic. |
| `navigation.py` | Pure decision logic: `decide(readings) → Command`. No hardware. |
| `sensors.py` | HC-SR04 driver. Real sensors on a Pi; `inf`/simulator elsewhere. |
| `motors.py` | Tank / skid-steer driver (DIR + PWM per side). Prints intent off-Pi. |
| `imu.py` | BNO086 IMU wrapper; provides `yaw()`. Disabled cleanly if absent. |
| `simulate.py` | Scripted off-hardware run into a front wall. |
| `test_navigation.py` | Unit tests for every branch of the turn logic. |
| `sensor_test.py` / `sensor_diagnostics.py` | Wiring / per-sensor debug helpers. |
| `imu_turn_test.py` | One-shot IMU spin test. |

---

## 8. Not yet implemented

- **Lane counting / "done" condition.** Nothing counts lanes, so the car doesn't
  know when the whole arena (≈ `ARENA_WIDTH_CM / LANE_WIDTH_CM` lanes) is covered
  and never stops on its own.
- **Closed-loop lane shift.** The sideways shift is still open-loop timed
  (`LANE_WIDTH_CM / DRIVE_CM_PER_S`); only the *turns* are closed-loop (IMU).
- **Left-side sensors** — add them to `config.SENSORS` and use them in
  `navigation.py` if you want symmetric wall-following.
