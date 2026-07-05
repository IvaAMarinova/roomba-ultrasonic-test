# Arena navigation — Roomba-style collecting & disposing car

A Raspberry Pi car that sweeps a closed, known-size **rectangular** arena in a
serpentine ("boustrophedon") pattern, collecting blocks in a front bucket
("багер") and dumping them out the **back** (waste-truck style) at a fixed
disposal **pit**. It drives with a **BNO086 IMU** (holding a straight heading and
dead-reckoning its position against the known arena size) and uses 5 ultrasonic
sensors (3 front, 2 right) only as a **safety fallback** near walls.

The car runs a small state machine — **DRIVING → TURNING → DISPOSING → DRIVING** —
in one continuous flow: it sweeps and collects, and whenever its estimated
position reaches the pit it stops, points its back at the pit, dumps, and resumes.

**Everything you tune lives in [`config.py`](config.py).** You should never have
to edit the logic files to change behaviour — read this doc, open `config.py`,
change numbers. The rest of this README explains what each number does.

> **Servos are not wired yet.** Block counting (collection servo) and the actual
> tip mechanism (disposal servo) are **placeholders** in [`actuators.py`](actuators.py)
> that only log — the whole state machine runs and is testable without them. See
> §7 for where they plug in.

---

## 1. The big picture: how the car drives

The car runs one loop (`main.py`): **read IMU + sensors → decide → drive**,
~20×/sec. Every tick it updates its estimated **pose** (x, y, heading) and prints
a full status line (see §6).

```
        ┌─ at the pit? (odometry position) ── yes ─► DISPOSE
        │                                            (face back at pit, dump)
read ──►│─ end of lane? (odometry distance,  ── yes ─► U-TURN into next lane
        │   or front sensor FALLBACK)                 (spin 90°, shift, spin 90°)
        │
        └─ else ─► CRUISE forward, holding the lane heading with the IMU
```

### The coordinate frame (where "position" is measured from)

The car starts in the **bottom-left corner** at `(START_X_CM, START_Y_CM)` = `(0, 0)`,
**facing straight along the arena length**. That facing becomes **heading 0**.

- **+y** = straight ahead, up a lane (along `ARENA_LENGTH_CM`).
- **+x** = to the car's right, across the arena (along `ARENA_WIDTH_CM`).

Position is **dead-reckoned**: heading comes from the IMU, and distance travelled
comes from `DRIVE_CM_PER_S × time`. There are **no wheel encoders**, so this drifts
over a run — measuring `DRIVE_CM_PER_S` accurately and having accurate turns is
what keeps the estimate good enough to find the pit.

### Cruising — DRIVING (the normal state)

While no turn/dispose is due, the car drives forward at `DRIVE_SPEED`. Instead of
following a wall, it **holds the lane's target heading with the IMU**: a steering
trim proportional to the heading error (`HEADING_HOLD_GAIN`, clamped to
`MAX_HEADING_TRIM`) keeps it going straight. If a wall gets within
`FRONT_SLOW_DISTANCE_CM` it drops to `SLOW_SPEED`.

> No IMU? Cruising falls back to **open-loop straight** (no trim), and the legacy
> right-wall follow is still available behind `USE_WALL_FOLLOW = True`.

### End of a lane — TURNING (the U-turn)

The lane is "over" when **odometry** says the car has driven
`ARENA_LENGTH_CM − LANE_END_MARGIN_CM` down it — **or**, as a **fallback**, when
the front sensors see a wall within `FRONT_STOP_DISTANCE_CM` (drift, or an
unexpected obstacle). Either way it does a **U-turn into the next lane** as one
atomic maneuver:

1. **Spin 90°** in place.
2. **Drive forward `LANE_WIDTH_CM`** — the sideways shift into the next lane.
3. **Spin 90° the same direction** — now facing back down the new lane.

Turn direction **alternates every U-turn**, starting with `SERPENTINE_FIRST_TURN`
(**right** from a bottom-left start), so the car steps `+x` across the arena and
snakes back and forth. After the maneuver the code folds its net effect (heading
reversed ~180°, one `LANE_WIDTH_CM` shift) back into the pose estimate.

### Reaching the pit — DISPOSING

When the pose comes within `PIT_ARRIVAL_RADIUS_CM` of the fixed pit
`(PIT_X_CM, PIT_Y_CM)`, the car enters **DISPOSING**: it rotates so its **back**
points at the pit (`DISPOSE_BACK_INTO_PIT`), holds for `DISPOSE_HOLD_S`, "dumps"
(placeholder), empties the bucket, and returns to DRIVING. It won't re-trigger
until it leaves and re-enters the pit zone.

> **Full-bucket gate (future):** today disposal triggers purely on **arriving** at
> the pit. Once the collection servo reports a real block count, flip
> `_should_dispose()` in `navigation.py` to gate on `collector.is_full()` — a
> one-line change, already marked with a TODO.

### Skid steer (how it physically moves)

It's a **tank / skid-steer** car: two sides (left, right), each with a direction
pin and a PWM speed pin. Forward = both sides forward. Turn in place = one side
forward, the other reverse. No steering servo. See `motors.py`.

---

## 2. How turning works (read this before tuning turns)

The 90° spins are the same closed-loop IMU turns as before, chosen by
`USE_IMU_TURN`:

### A) IMU closed-loop turns (`USE_IMU_TURN = True`, recommended)

The car spins and **watches its real heading**, stopping once it has actually
rotated `TURN_ANGLE_DEG` (minus `IMU_TURN_TOLERANCE_DEG` for coast). Accurate as
the battery drains or friction changes — no hand-tuned timing. It accumulates the
heading change sample-by-sample (wrap-safe around ±180°), ignores corrupted I2C
spikes (`IMU_GLITCH_MAX_STEP_DEG`), and has a hard timeout (`IMU_TURN_TIMEOUT_S`).

**Stall recovery (tire-tension boost).** Spinning in place scrubs the tires; if
after `IMU_TURN_BOOST_AFTER_S` the IMU shows **less than half** of `TURN_ANGLE_DEG`
turned, the code **bumps the spin speed** (by `IMU_TURN_BOOST_FACTOR`, capped at
1.0) to break through, and holds it. You'll see a `[u-turn] stall: ...` log line.

Disposal's "point the back at the pit" also uses the IMU — it rotates to the
absolute target heading and re-checks each poll, so a small overshoot self-corrects.

### B) Timed open-loop turns (`USE_IMU_TURN = False`, or no IMU)

The car spins at `TURN_SPEED` for `TURN_TIME_S` seconds and hopes that's 90°.
Simple, no IMU, but you must **measure `TURN_TIME_S`** and it drifts with battery.
For early bring-up only.

> The sideways lane shift (step 2 of the U-turn) is **always** open-loop timed as
> `LANE_WIDTH_CM / DRIVE_CM_PER_S` seconds. Both numbers must be measured.

---

## 3. Two run modes: drive test vs. navigation

`USE_SENSORS` in `config.py` picks what `main.py` does:

- **`USE_SENSORS = False` → open-loop drive test.** Runs the hardcoded
  `DRIVE_TEST_SEQUENCE` and **never reads the ultrasonics**. Use this *first* to
  confirm motors, H-bridge wiring, and turning. (`left`/`right` steps still use the
  IMU if `USE_IMU_TURN = True`.)
- **`USE_SENSORS = True` → full IMU + odometry navigation with disposal.** The real
  read → decide → drive loop described in §1.

---

## 4. Quickstart for the car

```bash
# 1. Bring-up: check motors + turning, no sensors needed.  (config.py: USE_SENSORS = False)
python3 main.py

# 2. Calibrate DRIVE_CM_PER_S: drive forward a known time, measure the distance,
#    divide -> set DRIVE_CM_PER_S. EVERYTHING (odometry, lane length, pit) rides on this.

# 3. Verify the heading convention on the bench: a "right" turn must make the car
#    step toward the arena (+x), not into the start wall. If mirrored, see §8.

# 4. Set the real arena, start pose, and PIT_X_CM / PIT_Y_CM.  Go live: USE_SENSORS = True
python3 main.py
```

Off the car (laptop, no Pi) everything still runs — motors print intent, sensors
read "nothing", the IMU reports unavailable (open-loop straight). Useful checks:

```bash
python3 test_navigation.py     # unit tests for the decision logic
python3 simulate.py            # time-stepped run: cruise -> odometry turn -> dispose
python3 sensor_test.py         # read & print all sensors (wiring check, on Pi)
python3 sensor_diagnostics.py  # per-sensor pins + raw samples (debug, on Pi)
python3 imu_turn_test.py       # spin once using the IMU and report the result
```

Stop the car any time with **Ctrl-C** — it stops the motors and cleans up GPIO.

---

## 5. `config.py` — the complete tuning reference

Grouped the same way as in the file.

### 5.1 Arena geometry & odometry

| Setting | What it does | When to change |
|---|---|---|
| `ARENA_WIDTH_CM` | Arena size across the lanes (the +x direction). | Match your real arena. |
| `ARENA_LENGTH_CM` | Arena size along each lane (the +y runs). Sets when odometry ends a lane. | Match your real arena. |
| `ROBOT_WIDTH_CM` | Physical width of the car. | Measure your car. |
| `LANE_WIDTH_CM` | Sideways shift per U-turn. **Must be ≤ `ROBOT_WIDTH_CM`** or it leaves gaps; a little less overlaps. | Tune coverage vs. speed. |
| `DRIVE_CM_PER_S` | **Measured** forward speed at `DRIVE_SPEED` (cm/s). Drives odometry, the lane shift, and pit arrival. | **Measure carefully** — the whole position estimate rides on it. |
| `LANE_END_MARGIN_CM` | Turn this far before the far wall (odometry trigger). Too small + drift → hits wall (fallback catches it); too big → unswept strip. | Tune. |

### 5.2 Start pose & sweep direction

| Setting | What it does |
|---|---|
| `START_X_CM` / `START_Y_CM` | Where the car begins, in the (x, y) frame. Default `(0, 0)` = bottom-left corner. **Place the car to match.** |
| `SERPENTINE_FIRST_TURN` | Which way the first U-turn curls: `"right"` from a bottom-left start (steps +x), `"left"` from a bottom-right start. Then it alternates. |

### 5.3 Disposal pit

| Setting | What it does |
|---|---|
| `PIT_X_CM` / `PIT_Y_CM` | Fixed pit centre in the (x, y) frame. **Set to the real pit location.** |
| `PIT_ARRIVAL_RADIUS_CM` | How close the pose must get to count as "at the pit". Make it comfortably larger than your expected odometry drift. |
| `DISPOSE_BACK_INTO_PIT` | `True` = rotate so the back faces the pit before dumping (needs the IMU). |
| `DISPOSE_HOLD_S` | Placeholder dwell while "dumping" (until the disposal servo lands). |

### 5.4 Collection (future servo)

| Setting | What it does |
|---|---|
| `COLLECTION_CAPACITY_BLOCKS` | The "bucket full" count. Wired to the collection servo later; today the count is a stub (always 0) so disposal triggers on pit arrival alone. |

### 5.5 Sensors (`SENSORS` dict + groupings)

Each sensor is `name: {"trig": <pin>, "echo": <pin>, "enabled": <bool>}` in **BCM**.

- **`trig` / `echo`** — match your wiring; keep clear of the motor pins (`12, 13, 16, 20`).
- **`enabled`** — `False` leaves that sensor's pins untouched and it always reports
  "no echo". Good for bring-up one at a time. (The two `right_*` sensors default to
  `False`; they're only needed for the legacy `USE_WALL_FOLLOW` mode.)
- **`FRONT_SENSORS`** — names that count as "front" for the **fallback** wall-stop.
  **`RIGHT_SENSORS`** — the "right" names for legacy wall-following.

### 5.6 Decision thresholds (centimetres)

| Setting | What it does |
|---|---|
| `FRONT_STOP_DISTANCE_CM` | **FALLBACK:** wall this close ahead → turn now, even if odometry disagrees. |
| `FRONT_SLOW_DISTANCE_CM` | Slow to `SLOW_SPEED` when a wall is this close. **Must be > `FRONT_STOP_DISTANCE_CM`.** |
| `RIGHT_WALL_DISTANCE_CM` | Right-side wall-present threshold (legacy wall-follow only). |
| `RIGHT_TARGET_DISTANCE_CM` | Desired right-wall gap (legacy wall-follow only). |

The front wall uses the **median** of the 3 front sensors, so one bad sensor can't
trigger or block the fallback on its own — at least two must agree.

### 5.7 Straight-line driving

| Setting | What it does |
|---|---|
| `USE_WALL_FOLLOW` | `False` (default) = **IMU heading-hold**. `True` = legacy right-wall trim (needs the `right_*` sensors enabled). |
| `HEADING_HOLD_GAIN` | Steer trim per degree of heading error. Too high = wobble; too low = slow to straighten. |
| `MAX_HEADING_TRIM` | Clamp on the heading-hold trim. |
| `STEER_CORRECTION_GAIN` / `MAX_STEER_TRIM` | Gain/clamp for the legacy wall-follow trim. |

### 5.8 Sensor reliability

| Setting | What it does |
|---|---|
| `SENSOR_MAX_RANGE_CM` / `SENSOR_MIN_RANGE_CM` | Readings outside this band are thrown out. |
| `SENSOR_TIMEOUT_S` | Give up waiting for an echo after this long. |
| `SENSOR_SAMPLES` | Median-of-N pings per read. Higher = steadier, slower. |
| `SOUND_SPEED_CM_PER_S` | Speed of sound for echo→distance. Rarely changed. |

### 5.9 Motion parameters

| Setting | What it does | Range |
|---|---|---|
| `DRIVE_SPEED` | Normal forward speed. | 0..1 |
| `SLOW_SPEED` | Speed when a wall is getting close. | 0..1 |
| `TURN_SPEED` | In-place rotation speed (base for IMU and timed spins, and disposal orient). | 0..1 |
| `TURN_TIME_S` | Seconds to spin 90° at `TURN_SPEED`. **Fallback only, no IMU.** | seconds |

### 5.10 IMU turning (closed-loop)

| Setting | What it does |
|---|---|
| `USE_IMU_TURN` | **Master switch.** `True` = measured-heading turns (auto-falls back to timed if the IMU is missing). Also gates whether the IMU is opened at all. |
| `TURN_ANGLE_DEG` | Target rotation for one spin (90°). |
| `IMU_TURN_TOLERANCE_DEG` | Stop this many degrees early for momentum. Raise if it overshoots. |
| `IMU_TURN_TIMEOUT_S` | Safety cap — a spin never runs longer than this. |
| `IMU_GLITCH_MAX_STEP_DEG` | Per-sample heading jumps bigger than this are ignored as corrupted reads. |
| `IMU_TURN_POLL_S` | How often to re-read heading during a spin. |
| `IMU_TURN_BOOST_AFTER_S` | Stall recovery: boost speed if under **half** turned by now. `None` disables. |
| `IMU_TURN_BOOST_FACTOR` | Multiplier applied to `TURN_SPEED` on a stall (capped at 1.0). |

### 5.11 Motors (`MOTORS` dict)

Each side is `name: {"dir": <pin>, "pwm": <pin>, "invert": <bool>}` in BCM.

- **`dir`** — direction (forward = HIGH). **`pwm`** — speed (0..100% duty).
- **`invert`** — flip if a wheel spins the wrong way (instead of re-wiring).
- `MOTOR_PWM_HZ` — PWM frequency. `MOTOR_DEADZONE` — commands below this = "stop".

### 5.12 Control loop & bring-up

| Setting | What it does |
|---|---|
| `CONTROL_LOOP_HZ` | How often the car reads + decides (≈20 Hz). Also sets the odometry `dt`. |
| `USE_SENSORS` | `False` = drive test, `True` = full navigation. |
| `DRIVE_TEST_SEQUENCE` | Scripted `(action, seconds)` maneuver for drive-test mode: `"forward"`, `"left"`, `"right"`, `"stop"`. |

---

## 6. Logging — what the car tells you

At startup, navigation prints a **banner**: arena size, start pose, pit location,
collection capacity, driving mode (heading-hold vs wall-follow), turn mode, and
whether real sensor hardware was found.

Then **every control tick** it prints one status line:

```
MODE=DRIVING   | pos=( 35.0, 156.0) hdg=-180.0 tgt=-180.0 lane=108.0 | front_left=... | yaw=  12.3 | blocks=0/10 | -> FORWARD    (cruising)
```

- **MODE** — DRIVING / TURNING / DISPOSING.
- **pos / hdg / tgt / lane** — estimated position, current & target heading, distance down this lane.
- **sensors** — every ultrasonic distance (`--` = no echo / out of range).
- **yaw** — raw IMU heading (`--` if no IMU this tick).
- **blocks** — collector count / capacity.
- **→ ACTION (reason)** — the decision and why (e.g. `odometry: lane length reached` vs `FALLBACK front wall 32cm`).

The U-turn and disposal maneuvers add their own `[u-turn] ...`, `[orient] ...`,
`[dispose] ...`, and `[disposer] DUMP ...` lines.

---

## 7. Files

| File | Purpose |
|---|---|
| `config.py` | **All** tunable values. Change behaviour here, never in the logic. |
| `main.py` | Control loop, U-turn / spin (IMU + stall boost), disposal maneuver, status logging. |
| `navigation.py` | Pure state machine: `decide(readings, yaw, dt) → Command`, odometry pose, modes. No hardware. |
| `actuators.py` | **Placeholders** for the future servos: `Collector` (block count/capacity) + `Disposer` (`dump()`). Log-only for now. |
| `sensors.py` | HC-SR04 driver. Real sensors on a Pi; `inf`/simulator elsewhere. |
| `motors.py` | Tank / skid-steer driver (DIR + PWM per side). Prints intent off-Pi. |
| `imu.py` | BNO086 IMU wrapper; provides `yaw()`. Disabled cleanly if absent. |
| `simulate.py` | Time-stepped off-hardware run: cruise → odometry turn → dispose. |
| `test_navigation.py` | Unit tests for every branch of the decision logic. |
| `sensor_test.py` / `sensor_diagnostics.py` | Wiring / per-sensor debug helpers. |
| `imu_turn_test.py` | One-shot IMU spin test. |

### Where the servos plug in (later)

- **Collection count** — call `Collector.add()` from the collection-servo code and
  read `Collector.is_full()`; then switch `_should_dispose()` in `navigation.py`
  from "arrived at pit" to "arrived **and** full" (TODO marked in the code).
- **Disposal actuation** — drive the tip servo inside `Disposer.dump()`. The
  DISPOSING maneuver (orient back-to-pit, hold, empty) already runs around it.

---

## 8. Gotchas & not-yet-implemented

- **Odometry drift is the core risk.** Position is pure dead-reckoning (no
  encoders). Measure `DRIVE_CM_PER_S`, keep turns accurate, and size
  `PIT_ARRIVAL_RADIUS_CM` for the drift you actually see over a full run. If drift
  is too large to find the pit by coordinates, a dedicated pit/edge sensor is the
  fallback plan.
- **Heading sign convention.** The code assumes **`turn_right` = +90° yaw** and that
  the car steps `+x` on its first turn. If your BNO086 yaw runs the other way, the
  first U-turn walks toward the start wall (−x) and the back-to-pit orient is
  mirrored. Verify on the bench (§4 step 3); as a quick workaround flip
  `SERPENTINE_FIRST_TURN`, or ask for the one-line convention fix in `complete_turn`.
- **Coverage / "done" condition.** The serpentine sweeps across the width but
  nothing counts lanes or declares the arena finished — the car keeps sweeping.
- **Closed-loop lane shift.** The sideways shift is still open-loop timed
  (`LANE_WIDTH_CM / DRIVE_CM_PER_S`); only the *turns* are closed-loop (IMU).
- **Left-side sensors** — add them to `config.SENSORS` and use them in
  `navigation.py` if you want symmetric fallback / wall-following.
```
