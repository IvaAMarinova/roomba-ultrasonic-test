# Arena navigation — Roomba-style collecting & disposing car

A Raspberry Pi car that sweeps a closed, known-size **rectangular** arena in a
serpentine ("boustrophedon") pattern, collecting blocks in a front bucket
("багер") and dumping them out the **back** (waste-truck style) at a fixed
disposal **pit**. It holds a straight heading with a **BNO086 IMU** and measures
its position against the **known walls** using **3 front ultrasonic sensors**
(distance to the end wall = how far down the lane it is), so localization survives
a bumpy floor. Wheel-time odometry is only a short bridge when a wall isn't seen.

The car runs a small state machine — **DRIVING → TURNING → DISPOSING → … → DONE** —
in one continuous flow: it sweeps and collects, and whenever its estimated position
reaches the pit it stops, backs its rear over the pit, dumps, and resumes; once all
lanes are swept it stops for good.

**Localization is wall-referenced, not time-based.** Because the arena has bumps
(wheels slip, so "distance = speed × time" drifts), position is measured against
the known walls instead: the front sensors' distance to the end wall *is* how far
down the lane the car is, and lane-counting gives which lane it's on. Wheel-time
odometry is only a short-gap **bridge** when a wall momentarily isn't seen. See §1.

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
        ┌─ at the pit? (wall-referenced position) ─ yes ─► DISPOSE
        │                                                  (face back at pit, dump)
read ──►│─ end of lane? (believed wall within standoff, ─ yes ─► U-TURN into next lane
        │   ≥2/3 agree, held a few ticks)                       (spin 90°, shift, spin 90°)
        │
        └─ else ─► CRUISE forward, holding the lane heading with the IMU
```

### Where "position" comes from

The car starts in the **bottom-left corner** at `(START_X_CM, START_Y_CM)` = `(0, 0)`,
**facing straight along the arena length**. That facing becomes **heading 0**.

- **+y** = straight ahead, up a lane (along `ARENA_LENGTH_CM`).
- **+x** = to the car's right, across the arena (along `ARENA_WIDTH_CM`).

Each coordinate has a **slip-immune** source:

| Quantity | Source | Bumps affect it? |
|---|---|---|
| **heading** | IMU yaw | no |
| **cross-lane `x`** | lane counting: `x = START_X + sweep × lane_index × LANE_WIDTH` | no |
| **along-lane `y`** | **front wall**: `y` derived from the measured gap to the end wall | no |
| _(fallback)_ | wheel-time odometry `DRIVE_CM_PER_S × time` — only bridges brief gaps | yes, but rarely used |

So `DRIVE_CM_PER_S` is **not** the primary position source anymore — it only bridges
the odd tick a wall isn't seen, provides a rough "where should the wall be" prior,
and times the sideways lane-shift. It can no longer drift you into the wrong place.

### Cruising — DRIVING (the normal state)

The car drives forward at `DRIVE_SPEED`, **holding the lane's target heading with
the IMU** (steering trim ∝ heading error, `HEADING_HOLD_GAIN` clamped to
`MAX_HEADING_TRIM`) — this keeps it square to the wall, which is what makes the
wall distance read cleanly. If a wall is within `FRONT_SLOW_DISTANCE_CM` it slows to
`SLOW_SPEED`. Every tick it derives `y` from the front wall (when seen) or bridges
on odometry, and logs which (`src=WALL` / `src=BRIDGE`).

> No IMU? Cruising falls back to **open-loop straight** (no trim), and the legacy
> right-wall follow is still available behind `USE_WALL_FOLLOW = True`.

### Trusting the wall — rejecting bumps, blocks, and misses

A close front reading is only believed to be the **end wall** when all three hold:

1. **≥`FRONT_AGREE_MIN_COUNT` of 3 front sensors agree** within `FRONT_AGREE_TOL_CM`.
   A real wall spans the whole (edge-to-edge) front; a narrow block/bump shows up on
   one sensor and is outvoted.
2. **It's near where a wall is expected** (odometry prior, within `WALL_EXPECT_TOL_CM`).
   A wide object mid-lane can't be the end wall, so it's treated as a block to
   collect (slow, drive on), not the lane end.
3. **It persists `WALL_PERSIST_TICKS` ticks.** Kills single-frame glitches.

If the wall momentarily **drops out** (angled/specular miss), `y` coasts on odometry
for up to `BRIDGE_MAX_S` while the IMU keeps the car square so the wall re-appears.

### End of a lane — TURNING (the U-turn)

The lane is "over" when a **believed wall** (all three rules above) is within
`FRONT_STOP_DISTANCE_CM` — a fixed, measured standoff that stays consistent across
bumps. If the front sensors give **no** agreed wall for the whole lane, an odometry
**backstop** (`ARENA_LENGTH_CM − LANE_END_MARGIN_CM`) turns anyway so it can't drive
blind forever. Either way it does a **U-turn into the next lane** as one atomic
maneuver:

1. **Spin 90°** in place.
2. **Drive forward `LANE_WIDTH_CM`** — the sideways shift into the next lane.
3. **Spin 90° the same direction** — now facing back down the new lane.

Turn direction **alternates every U-turn**, starting with `SERPENTINE_FIRST_TURN`
(**right** from a bottom-left start), so the car steps `+x` across the arena and
snakes back and forth. After the maneuver the code reverses the target heading and
**increments the lane index** (which is what sets the new cross-lane `x` — exactly,
no dependence on how far the physical shift actually went).

### Reaching the pit — DISPOSING

The pit sits in the **middle of the start wall** (`PIT_X_CM ≈ WIDTH/2, PIT_Y_CM ≈ 0`)
and is **small — about the size of the car — so the rear is placed directly over
it**, not just aimed at it. Because it's on the start wall and each lane touches
that wall at its own `x`, the sweep **passes the pit exactly once** — on the one
lane whose `x` matches the pit. When the pose comes within `PIT_ARRIVAL_RADIUS_CM`
of the pit, the car enters **DISPOSING** and runs a 5-step maneuver:

1. Turn so its **back** faces the pit (`DISPOSE_BACK_INTO_PIT`).
2. **Reverse `DISPOSE_REVERSE_CM`** (slowly, `DISPOSE_REVERSE_SPEED`) to seat the
   rear over the pit — tune so the rear overhangs but the drive wheels stay on the
   edge.
3. Hold `DISPOSE_HOLD_S` and **dump** (placeholder until the servo lands).
4. **Pull `DISPOSE_REVERSE_CM` forward** to get clear of the pit.
5. Re-orient to the lane heading and return to DRIVING.

It won't re-trigger until it leaves and re-enters the pit zone.

### Finishing up — DONE + a final dump

The arena is `NUM_LANES` lanes wide (config, by default `ceil(ARENA_WIDTH_CM /
LANE_WIDTH_CM)`). The car counts lanes as it sweeps; when it finishes the last one
(`lane_index` reaches `NUM_LANES − 1`) it enters **DONE**. Because the sweep passed
the pit only once (mid-way), blocks collected afterwards are still aboard, so DONE
triggers **one final trip back to the pit**:

1. Drive to the **start wall** (the pit's side), wherever the sweep ended.
2. Drive to the **left side wall** (exact `x` from that wall), then move `PIT_X_CM`
   across to the pit's **middle** — the one spot walls can't mark, so it's measured
   from the side wall (short hug; the car-sized pit + reverse gives tolerance).
3. Face the start wall so the back points at the pit, **reverse in and dump**, then
   stop for good.

Override `NUM_LANES` in config to sweep fewer/more lanes than the plain division gives.

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

# 2. Check the front sensors: on a flat wall all three should read ~the same
#    (they "agree"); wave a small object in front of ONE and confirm the others
#    still see the wall. Position is wall-referenced, so this is what matters most.

# 3. Set DRIVE_CM_PER_S roughly (drive a known time, measure, divide). It's only a
#    fallback/bridge now, so a ballpark value is fine.

# 4. Verify the heading convention on the bench: a "right" turn must step the car
#    toward the arena (+x), not into the start wall. If mirrored, see §8.

# 5. Set the real arena, start pose, and PIT_X_CM / PIT_Y_CM.  Go live: USE_SENSORS = True
python3 main.py
```

Off the car (laptop, no Pi) everything still runs — motors print intent, sensors
read "nothing", the IMU reports unavailable (open-loop straight). Useful checks:

```bash
python3 test_navigation.py     # unit tests for the decision logic
python3 simulate.py            # time-stepped run: wall-referenced cruise -> wall turn -> dispose
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
| `ARENA_LENGTH_CM` | Arena size along each lane (the +y runs). The wall reference measures against this. | Match your real arena. |
| `ROBOT_WIDTH_CM` | Physical width of the car. | Measure your car. |
| `LANE_WIDTH_CM` | Sideways shift per U-turn. **Must be ≤ `ROBOT_WIDTH_CM`** or it leaves gaps; a little less overlaps. | Tune coverage vs. speed. |
| `NUM_LANES` | How many lanes = a full sweep; the car STOPS (DONE) after finishing the last one. Default `ceil(ARENA_WIDTH_CM / LANE_WIDTH_CM)`. | Override to sweep fewer/more lanes. |
| `DRIVE_CM_PER_S` | Rough forward speed at `DRIVE_SPEED` (cm/s). **Fallback/bridge only** now (position is wall-referenced) — bridges sensor dropouts, the "expected wall" prior, and the lane-shift timing. | Measure once, roughly; it no longer needs to be exact. |
| `LANE_END_MARGIN_CM` | Odometry **backstop**: turn this far before the far wall *only if a wall is never seen* the whole lane. | Tune. |

### 5.2 Start pose & sweep direction

| Setting | What it does |
|---|---|
| `START_X_CM` / `START_Y_CM` | Where the car begins, in the (x, y) frame. Default `(0, 0)` = bottom-left corner. **Place the car to match.** |
| `SERPENTINE_FIRST_TURN` | Which way the first U-turn curls: `"right"` from a bottom-left start (steps +x), `"left"` from a bottom-right start. Then it alternates. |

### 5.3 Disposal pit

| Setting | What it does |
|---|---|
| `PIT_X_CM` / `PIT_Y_CM` | Fixed pit centre in the (x, y) frame. **Set to the real pit location.** |
| `PIT_ARRIVAL_RADIUS_CM` | How close the (wall-referenced) pose must get to count as "at the pit" during the sweep. Size it for the residual error you see near the start wall. |
| `DISPOSE_BACK_INTO_PIT` | `True` = rotate so the back faces the pit before reversing (needs the IMU). |
| `DISPOSE_REVERSE_CM` | How far to reverse (after orienting) to seat the rear over the small, car-sized pit. **Tune** so the rear overhangs but the drive wheels stay on the edge. Pulled forward again after dumping. |
| `DISPOSE_REVERSE_SPEED` | Speed (0..1) for that reverse/pull — slow, for precise placement. |
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
- **`FRONT_SENSORS`** — the 3 front names, the **primary** position reference.
  **Mount them spread edge-to-edge, outboard of the bucket, level, above block
  height** (see the comment in `config.py`). **`RIGHT_SENSORS`** — legacy wall-follow.

### 5.6 Decision thresholds (centimetres)

| Setting | What it does |
|---|---|
| `FRONT_STOP_DISTANCE_CM` | **PRIMARY:** turn when the believed end wall is this close — a fixed, measured standoff. |
| `FRONT_SLOW_DISTANCE_CM` | Slow to `SLOW_SPEED` when a wall/object is this close. **Must be > `FRONT_STOP_DISTANCE_CM`.** |
| `RIGHT_WALL_DISTANCE_CM` | Right-side wall-present threshold (legacy wall-follow only). |
| `RIGHT_TARGET_DISTANCE_CM` | Desired right-wall gap (legacy wall-follow only). |

### 5.7 Wall-detection fusion (reject blocks/bumps and misses)

These gate when a close front reading is believed to be the **end wall** (see §1
"Trusting the wall"):

| Setting | What it does |
|---|---|
| `FRONT_AGREE_TOL_CM` | Front readings within this of each other count as "agreeing". |
| `FRONT_AGREE_MIN_COUNT` | How many must agree (K). With 3 sensors, `2` = "the median is a real wall, not one stray sensor". |
| `WALL_EXPECT_TOL_CM` | How far the odometry prior may disagree with the measured wall and still trust it. **Generous** (odometry is rough); too tight rejects real walls after drift, too loose lets a wide mid-lane object be called the lane end. |
| `WALL_PERSIST_TICKS` | Consecutive ticks a close wall must hold before turning (kills single-frame glitches). |
| `BRIDGE_MAX_S` | How long along-lane position may coast on odometry when the wall drops out. |

### 5.8 Straight-line driving

| Setting | What it does |
|---|---|
| `USE_WALL_FOLLOW` | `False` (default) = **IMU heading-hold**. `True` = legacy right-wall trim (needs the `right_*` sensors enabled). |
| `HEADING_HOLD_GAIN` | Steer trim per degree of heading error. Too high = wobble; too low = slow to straighten. |
| `MAX_HEADING_TRIM` | Clamp on the heading-hold trim. |
| `STEER_CORRECTION_GAIN` / `MAX_STEER_TRIM` | Gain/clamp for the legacy wall-follow trim. |

### 5.9 Sensor reliability

| Setting | What it does |
|---|---|
| `SENSOR_MAX_RANGE_CM` / `SENSOR_MIN_RANGE_CM` | Readings outside this band are thrown out. |
| `SENSOR_TIMEOUT_S` | Give up waiting for an echo after this long. |
| `SENSOR_SAMPLES` | Median-of-N pings per read. Higher = steadier, slower. |
| `SOUND_SPEED_CM_PER_S` | Speed of sound for echo→distance. Rarely changed. |

### 5.10 Motion parameters

| Setting | What it does | Range |
|---|---|---|
| `DRIVE_SPEED` | Normal forward speed. | 0..1 |
| `SLOW_SPEED` | Speed when a wall is getting close. | 0..1 |
| `TURN_SPEED` | In-place rotation speed (base for IMU and timed spins, and disposal orient). | 0..1 |
| `TURN_TIME_S` | Seconds to spin 90° at `TURN_SPEED`. **Fallback only, no IMU.** | seconds |

### 5.11 IMU turning (closed-loop)

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

### 5.12 Motors (`MOTORS` dict)

Each side is `name: {"dir": <pin>, "pwm": <pin>, "invert": <bool>}` in BCM.

- **`dir`** — direction (forward = HIGH). **`pwm`** — speed (0..100% duty).
- **`invert`** — flip if a wheel spins the wrong way (instead of re-wiring).
- `MOTOR_PWM_HZ` — PWM frequency. `MOTOR_DEADZONE` — commands below this = "stop".

### 5.13 Control loop & bring-up

| Setting | What it does |
|---|---|
| `CONTROL_LOOP_HZ` | How often the car reads + decides (≈20 Hz). Also sets the `dt` used for bridging. |
| `USE_SENSORS` | `False` = drive test, `True` = full navigation. |
| `DRIVE_TEST_SEQUENCE` | Scripted `(action, seconds)` maneuver for drive-test mode: `"forward"`, `"left"`, `"right"`, `"stop"`. |

---

## 6. Logging — what the car tells you

At startup, navigation prints a **banner**: arena size, start pose, pit location,
collection capacity, driving mode (heading-hold vs wall-follow), turn mode, and
whether real sensor hardware was found.

Then **every control tick** it prints one status line:

```
MODE=DRIVING   | pos=(  35.0, 156.0) hdg=-180.0 tgt=-180.0 lane#1 ld=144.0 src=WALL   wall=144.0(x3) | front_left=144.0 ... | yaw= -179.8 | blocks=0/10 | -> FORWARD    (cruising)
```

- **MODE** — DRIVING / TURNING / DISPOSING.
- **pos / hdg / tgt** — estimated position, current & target heading.
- **lane# / ld** — lane index (→ cross-lane `x`) and along-lane distance.
- **src** — where along-lane position came from: `WALL` (believed wall) or `BRIDGE` (odometry).
- **wall=NN(xK)** — believed wall distance and how many front sensors agreed (`--` = none).
- **sensors** — every raw ultrasonic distance (`--` = no echo / out of range).
- **yaw** — raw IMU heading (`--` if no IMU this tick). **blocks** — collector count / capacity.
- **→ ACTION (reason)** — the decision and why (e.g. `wall 32cm (x3 agree)` vs `odometry backstop (no wall seen)`).

The U-turn and disposal maneuvers add their own `[u-turn] ...`, `[orient] ...`,
`[dispose] ...`, and `[disposer] DUMP ...` lines.

---

## 7. Files

| File | Purpose |
|---|---|
| `config.py` | **All** tunable values. Change behaviour here, never in the logic. |
| `main.py` | Control loop, U-turn / spin (IMU + stall boost), disposal maneuver, status logging. |
| `navigation.py` | Pure state machine: `decide(readings, yaw, dt) → Command`, wall-referenced pose + fusion, modes. No hardware. |
| `actuators.py` | **Placeholders** for the future servos: `Collector` (block count/capacity) + `Disposer` (`dump()`). Log-only for now. |
| `sensors.py` | HC-SR04 driver. Real sensors on a Pi; `inf`/simulator elsewhere. |
| `motors.py` | Tank / skid-steer driver (DIR + PWM per side). Prints intent off-Pi. |
| `imu.py` | BNO086 IMU wrapper; provides `yaw()`. Disabled cleanly if absent. |
| `simulate.py` | Time-stepped off-hardware run (synthesizes wall readings): wall-referenced cruise → wall turn → dispose. |
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

- **Sensor mounting is now the thing that matters most.** Position depends on the
  front sensors seeing the walls. They must be **level, above block height, spread
  edge-to-edge and outboard of the bucket**, aimed square at the walls. A sensor
  pointed at the floor, or looking into its own bucket, breaks the wall reference no
  matter how good the code is (§1 "Trusting the wall"). Verify on the bench: on a
  flat surface all three should agree; wave a small object in front of one and it
  should **not** trigger a turn.
- **Specular miss.** Ultrasonics off a smooth wall at an angle bounce away. IMU
  heading-hold keeps the car square (good returns); when the car does yaw on a bump
  and the wall drops out, position **bridges** on odometry for `BRIDGE_MAX_S` until
  it re-appears. Long bridges over a very bumpy floor still drift — that's what the
  odometry backstop and a generous `PIT_ARRIVAL_RADIUS_CM` are for.
- **Heading sign convention.** The code assumes **`turn_right` = +90° yaw** and that
  the car steps `+x` on its first turn. If your BNO086 yaw runs the other way, the
  first U-turn walks the wrong way and the back-to-pit orient is mirrored. Verify on
  the bench (§4 step 3); quick workaround: flip `SERPENTINE_FIRST_TURN`.
- **Coverage assumes a clean lane count.** DONE fires after `NUM_LANES` lanes; if
  the arena width isn't a tidy multiple of `LANE_WIDTH_CM`, or lanes drift, the last
  strip may be partially covered or the car may stop a little early/late. Tune
  `NUM_LANES` / `LANE_WIDTH_CM` for your arena.
- **Closed-loop lane shift.** The sideways shift is still open-loop timed
  (`LANE_WIDTH_CM / DRIVE_CM_PER_S`); only the *turns* are closed-loop (IMU). Cross-
  lane `x` comes from lane-counting, so a slightly-off shift doesn't corrupt `x`,
  but a badly-off shift means the physical lane isn't where `x` says.
- **Left-side sensors** — add them to `config.SENSORS` and use them in
  `navigation.py` if you want to cross-check `x` against the side walls in edge lanes.
```
