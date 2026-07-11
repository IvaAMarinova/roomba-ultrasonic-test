import math

# ---------------------------------------------------------------------------
# Arena geometry (known and fixed for the competition).
# ---------------------------------------------------------------------------
# ARENA_WIDTH_CM = 210.0        # across the lanes (the car steps sideways here)
# ARENA_LENGTH_CM = 300.0       # along each lane (the long axis the car runs)
ARENA_WIDTH_CM = 150.0        # across the lanes (the car steps sideways here)
ARENA_LENGTH_CM = 212.0       # along each lane (the long axis the car runs)

ROBOT_WIDTH_CM = 50.0         # physical width of the car

# How far the car shifts sideways at each end-of-lane U-turn. For full coverage
# with no gaps this must be <= ROBOT_WIDTH_CM; a little less gives overlap.
# 210 / 45 ~= 5 lanes. Bigger = wider shift (faster, but leaves unswept strips
# once it exceeds the car width).
LANE_WIDTH_CM = 35.0

# How many lanes make up a full sweep of the arena = how wide the arena is
# divided by how far we step each lane (rounded UP so the far edge is covered).
# The car counts lanes as it goes and STOPS once it has finished lane NUM_LANES-1
# (coverage complete). Override the number here if you want to sweep fewer/more
# lanes than the plain division gives (e.g. partial coverage, or extra overlap).
NUM_LANES = math.ceil(ARENA_WIDTH_CM / LANE_WIDTH_CM)   # e.g. 210 / 35 = 6

# Forward travel speed (cm/s) is CALIBRATED and DERIVED near the motion parameters
# below (FULL_SPEED_CM_PER_S -> DRIVE_CM_PER_S), so it tracks DRIVE_SPEED instead of
# being a separate hardcoded number. See that section.

# ---------------------------------------------------------------------------
# Start pose and coordinate frame.
#   The car starts in the BOTTOM-LEFT corner, facing along the arena length. We
#   work in a "start-relative" frame: at startup the current IMU yaw is recorded
#   as heading 0 (the initial facing), and (START_X_CM, START_Y_CM) is where the
#   car begins. +y is straight ahead (up the first lane, along ARENA_LENGTH_CM),
#   +x is to the car's right (across ARENA_WIDTH_CM). Odometry integrates motion
#   in this frame; keep the physical placement consistent with these values.
# ---------------------------------------------------------------------------
START_X_CM = 0.0                    # bottom-left corner, left edge
START_Y_CM = 0.0                    # bottom-left corner, start wall

# Serpentine sweep direction. From the bottom-left corner facing +y, the first
# U-turn at the far wall must curl RIGHT so each lane steps +x across the arena
# (then it alternates right/left/right/... for a full-width snake).
SERPENTINE_FIRST_TURN = "right"     # "right" from bottom-left, "left" from bottom-right

# ---------------------------------------------------------------------------
# Disposal pit -- in the MIDDLE of the start wall (y=0), known fixed location.
#   The pit is SMALL (about the size of the car), so we position the car's REAR
#   directly over it: turn so the back faces the pit, REVERSE DISPOSE_REVERSE_CM
#   to seat the rear over it, dump (waste-truck style), pull forward off it.
#   The sweep passes the pit ONCE (it's on the start wall), and after the whole
#   arena is swept the car returns here for a final dump (see main._return_to_pit).
#   SET THESE to the real pit location for the competition arena.
# ---------------------------------------------------------------------------
PIT_X_CM = ARENA_WIDTH_CM / 2.0     # pit centre X: middle of the start wall (TODO: confirm)
PIT_Y_CM = 0.0                      # pit centre Y: on the start wall (y=0) (TODO: confirm)
# How close (cm) to the pit centre counts as "arrived". The pit is ON the start
# wall, and the car can't approach a wall closer than FRONT_STOP_DISTANCE_CM, so
# this MUST be >= FRONT_STOP_DISTANCE_CM or the end-of-lane turn fires first and the
# car never disposes. Disposal is also gated to the pit's own lane, so a generous
# radius can't false-trigger on a neighbouring lane.
PIT_ARRIVAL_RADIUS_CM = 50.0
DISPOSE_BACK_INTO_PIT = True        # True = orient the back toward the pit before reversing
# How far to reverse (after orienting) to place the rear directly over the small
# pit, and how fast. TUNE on the real setup so the rear OVERHANGS the pit but the
# drive wheels stay on the edge (the pit is roughly car-sized). The car pulls the
# same distance forward again after dumping to get clear before resuming.
DISPOSE_REVERSE_CM = 20.0
DISPOSE_REVERSE_SPEED = 0.3         # slow, for precise placement (0..1)
DISPOSE_HOLD_S = 2.0                # dwell with rear door open while balls fall out

# ---------------------------------------------------------------------------
# Collection (future sensor).
#   The collection servo will report how many blocks are in the bucket; when it
#   reaches COLLECTION_CAPACITY_BLOCKS the bucket is "full". For now the count is
#   a stub (see actuators.Collector) and disposal triggers purely on reaching the
#   pit -- wiring is a one-line change once the servo exists.
# ---------------------------------------------------------------------------
COLLECTION_CAPACITY_BLOCKS = 10     # TODO: real bucket capacity; count comes from the collection servo later

# ---------------------------------------------------------------------------
# Front scoop servo.
#   The shovel exceeds the max length when down, but the regulation allows that
#   as long as the run starts with the scoop raised. On launch the scoop is
#   commanded up, held for FRONT_SERVO_START_UP_S (0 = lower immediately), then
#   lowered to the collecting position. While driving it lifts to
#   FRONT_SERVO_UP_PULSE_MS every FRONT_SERVO_INTERVAL_S, holds FRONT_SERVO_HOLD_S,
#   then returns to FRONT_SERVO_DOWN_PULSE_MS. See actuators.FrontServo.
# ---------------------------------------------------------------------------
FRONT_SERVO_PIN = 18                # BCM pin for the front servo PWM signal
FRONT_SERVO_DOWN_PULSE_MS = 0.780   # resting / collecting pulse width
FRONT_SERVO_UP_PULSE_MS = 1.840     # raised pulse width
FRONT_SERVO_MOVE_S = 0.80           # seconds for full down<->up travel (0 = instant jump)
FRONT_SERVO_RAMP_STEP_S = 0.02      # update interval while ramping
FRONT_SERVO_START_UP_S = 0.0        # seconds to hold scoop up at launch before lowering
FRONT_SERVO_INTERVAL_S = 20.0       # raise the front scoop this often (seconds of driving)
FRONT_SERVO_HOLD_S = 1.0            # dwell at the top before returning down

# ---------------------------------------------------------------------------
# Back dump door servo.
#   Opens the rear door so collected balls fall into the pit. Starts closed;
#   CLOSED and OPEN pulse widths are independent values; either may be larger.
#   Calibrate with back_servo_calibrate.py. See actuators.BackServo / Disposer.
# ---------------------------------------------------------------------------
BACK_SERVO_PIN = 19                 # BCM pin for the rear door servo PWM signal
BACK_SERVO_CLOSED_PULSE_MS = 2.45   # door closed (balls retained) -- calibrate
BACK_SERVO_OPEN_PULSE_MS = 1.90     # door open (dump) -- calibrate
BACK_SERVO_MOVE_S = 0.5             # seconds for full closed<->open travel
BACK_SERVO_RAMP_STEP_S = 0.02       # update interval while ramping

# ---------------------------------------------------------------------------
# Ultrasonic sensor layout.
#   5x HC-SR04 positions: 3 front, 2 back. (No side sensors -- localisation uses
#   the FRONT wall for position + the IMU for heading; there is no side-wall logic.)
#   Each entry: logical name -> {TRIG pin, ECHO pin, enabled} in BCM numbering.
#   Adjust the pin numbers to match your wiring.
#   "enabled": False -> that sensor is never read (its GPIO is left untouched)
#   and its distance always reports as "no echo". Use it to bring sensors up one
#   at a time, or to ignore one that isn't wired / is faulty.
#
#   The 3 FRONT sensors are the PRIMARY position reference (distance to the end
#   wall = how far down the lane we are), used both during the sweep and when
#   driving up to the walls on the return-to-pit phase. Mount them:
#     * spread edge-to-edge across the front (left-edge / center / right-edge) and
#       OUTBOARD of the collection bucket, so no sensor stares into its own scoop
#       and the outer pair gives a wide "is it a full wall or just a block?" baseline,
#     * LEVEL, aimed straight ahead, and at a height ABOVE the blocks but BELOW the
#       top of the arena walls -- high enough that low blocks/bumps are not seen, but
#       not so high the beam shoots over a low wall and misses it.
#   The 2 BACK sensors are earmarked for future reverse / disposal assistance
#   (backing the rear over the small pit); the nav logic does not read them yet.
# ---------------------------------------------------------------------------
# NOTE: pins 12, 13, 16, 20 are used by the motors (see MOTORS below) and 2, 3 by
# the IMU's I2C, so the sensors are wired clear of them. Every ENABLED sensor must
# have unique trig/echo pins. Change all of these to match your actual wiring.
SENSORS = {
    # Front -- ENABLED (primary position reference).
    "front_left":   {"trig": 6,  "echo": 5,  "enabled": True},   # left edge, outboard of bucket
    "front_center": {"trig": 27, "echo": 22, "enabled": True},   # centre
    "front_right":  {"trig": 23, "echo": 24, "enabled": True},   # right edge, outboard of bucket
    # Back -- future: reverse / disposal assistance.
    "back_left":    {"trig": 1,  "echo": 25, "enabled": True},   # back, left
    "back_right":   {"trig": 7,  "echo": 8,  "enabled": True},   # back, right
}

# Logical groupings used by the navigation logic.
FRONT_SENSORS = ("front_left", "front_center", "front_right")
BACK_SENSORS = ("back_left", "back_right")

# ---------------------------------------------------------------------------
# Decision thresholds (centimetres).
#   End-of-lane is decided PRIMARILY by the front wall: we turn when the car is a
#   fixed standoff (FRONT_STOP_DISTANCE_CM) from the end wall. This is a real
#   measured gap, so the standoff stays consistent even when wheel slip on bumps
#   throws off the time-based estimate. Odometry is only the backstop (used if all
#   front sensors drop out -- see LANE_END_MARGIN_CM).
# ---------------------------------------------------------------------------
FRONT_STOP_DISTANCE_CM = 40.0    # PRIMARY: turn when the end wall is this close (fixed standoff)
FRONT_SLOW_DISTANCE_CM = 50.0    # start slowing down / preparing to turn
# NOTE: STOP must be < SLOW, and both are clearances (small), NOT sensor range.

# ---------------------------------------------------------------------------
# Wall-detection fusion (reject blocks/bumps and angled misses).
#   A close front reading is only believed to be the END WALL when:
#     * at least FRONT_AGREE_MIN_COUNT of the enabled front sensors agree within
#       FRONT_AGREE_TOL_CM of each other (a narrow block can't fool the wide,
#       edge-to-edge spread -- a real wall is seen the same by all), AND
#     * odometry says we are within WALL_EXPECT_TOL_CM of where the end wall is
#       expected (rejects a mid-lane object being called the lane end), AND
#     * the above holds for WALL_PERSIST_TICKS consecutive ticks (rejects a
#       single-frame glitch).
#   If the front wall momentarily drops out (angled/specular miss), along-lane
#   position coasts on odometry until the wall re-appears; the odometry backstop
#   (LANE_END_MARGIN_CM) still ends the lane if the wall is never seen.
# ---------------------------------------------------------------------------
FRONT_AGREE_TOL_CM = 15.0        # front readings within this of each other "agree"
FRONT_AGREE_MIN_COUNT = 2        # need at least this many agreeing (K of 3)
WALL_EXPECT_TOL_CM = 70.0        # how far odometry may disagree with the wall and still trust it (generous: odometry is rough)
WALL_PERSIST_TICKS = 3           # consecutive ticks the wall-stop must hold before turning
WALL_HEADING_ALIGN_DEG = 30.0    # only fuse / stop on a front wall when square to the lane heading

# ---------------------------------------------------------------------------
# Cross-lane (x) position.
#   x comes purely from LANE COUNTING: x = START_X + sweep_sign * lane_index *
#   LANE_WIDTH, stepped one lane per U-turn. There is no side-wall correction --
#   staying centred in the lane is the IMU heading-hold's job (it keeps the car
#   square so each straight run and each 90-degree turn tracks the lane geometry).
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Sensor reliability / read settings.
# ---------------------------------------------------------------------------
SENSOR_MAX_RANGE_CM = 400.0      # HC-SR04 practical maximum
SENSOR_MIN_RANGE_CM = 2.0        # HC-SR04 practical minimum
SENSOR_TIMEOUT_S = 0.03          # give up waiting for an echo after this long
SENSOR_SAMPLES = 3               # median-of-N samples per read to reject noise
SOUND_SPEED_CM_PER_S = 34300.0   # speed of sound, used to convert echo time

# ---------------------------------------------------------------------------
# Motion parameters.
# ---------------------------------------------------------------------------
DRIVE_SPEED = 0.3                # nominal forward speed (0..1 duty)
SLOW_SPEED = 0.2                 # forward speed when an obstacle is getting close
TURN_SPEED = 0.1                 # in-place rotation speed

# Physical speed calibration -- the ONE thing to measure. FULL_SPEED_CM_PER_S is how
# fast the car actually travels at full duty (speed 1.0), in cm/s. The odometry cm/s
# at any commanded duty is FULL_SPEED_CM_PER_S * duty (the model already assumes speed
# is ~proportional to duty), so DRIVE_CM_PER_S is DERIVED from DRIVE_SPEED below and
# tracks it automatically -- change DRIVE_SPEED and the distance estimate follows, no
# separate re-measure. To calibrate: drive at DRIVE_SPEED for a known time, cm/s =
# distance/time, then FULL_SPEED_CM_PER_S = that / DRIVE_SPEED.
FULL_SPEED_CM_PER_S = 153.334     # e.g. 30 cm/s at DRIVE_SPEED 0.6 -> 30 / 0.6 = 50

# cm/s at the normal cruise speed. DERIVED (not hardcoded). It is only a fallback/
# bridge value now that position is wall-referenced: it bridges along-lane position
# for the brief ticks a wall isn't seen, gives the rough "where should the wall be"
# prior (WALL_EXPECT_TOL_CM), and times the LANE_WIDTH sideways shift in the U-turn.
DRIVE_CM_PER_S = FULL_SPEED_CM_PER_S * DRIVE_SPEED

# ---------------------------------------------------------------------------
# Straight-line driving: IMU heading-hold.
#   While DRIVING, the car cruises forward and trims its steering to hold the
#   lane's target heading using the IMU. HEADING_HOLD_GAIN turns a heading error
#   (deg) into a steer trim, clamped to +/-MAX_HEADING_TRIM. Positive steer =
#   toward the car's right. (With no IMU, the trim is 0 -> open-loop straight.)
# ---------------------------------------------------------------------------
HEADING_HOLD_GAIN = 0.02          # steer trim per degree of heading error
MAX_HEADING_TRIM = 0.4            # clamp on the heading-hold steering trim

# ---------------------------------------------------------------------------
# Odometry end-of-lane BACKSTOP.
#   Only used if the front sensors give no agreed wall reading at all (total
#   dropout): once dead-reckoned lane distance reaches ARENA_LENGTH_CM -
#   LANE_END_MARGIN_CM the car turns anyway, so it can never run forever blind.
#   In normal operation the wall trigger (above) fires first.
# ---------------------------------------------------------------------------
LANE_END_MARGIN_CM = 40.0         # backstop: turn this far before the far wall if the wall is never seen

# Seconds to rotate 90 degrees in place at TURN_SPEED. Used as the FALLBACK when
# no IMU is available (see USE_IMU_TURN below). Measure it on the actual car.
TURN_TIME_S = 4

# ---------------------------------------------------------------------------
# IMU-based (closed-loop) turning.
#   With a BNO086 IMU present, an end-of-lane spin rotates until the *measured*
#   heading change reaches TURN_ANGLE_DEG, instead of spinning blindly for
#   TURN_TIME_S. This removes the dependence on the hand-tuned TURN_TIME_S and
#   stays accurate as battery voltage / floor friction change.
#
#   USE_IMU_TURN is the master ON/OFF switch for this:
#     True  -> turn by IMU heading (auto-falls back to timed if the IMU is
#              missing or fails to initialise -- nothing breaks without it).
#     False -> IMU is never touched; ALL spins use the timed TURN_TIME_S spin.
# ---------------------------------------------------------------------------
USE_IMU_TURN = True              # True = IMU heading-feedback turns, False = timed (TURN_TIME_S)
TURN_ANGLE_DEG = 90.0            # target rotation for one end-of-lane spin
IMU_TURN_TOLERANCE_DEG = 3.0     # stop this many deg early to allow for coast/momentum
ORIENT_SKIP_DEG = 15.0           # skip dispose / return spins when already within this of target
IMU_TURN_TIMEOUT_S = TURN_TIME_S * 2.5  # safety cap: never spin longer than this
IMU_GLITCH_MAX_STEP_DEG = 45.0   # per-sample heading jumps larger than this are
                                 # treated as corrupted I2C reads and ignored
IMU_TURN_POLL_S = 0.01           # how often to re-read heading during a spin

# Stall recovery during a spin. If after IMU_TURN_BOOST_AFTER_S of turning the
# IMU still shows we've covered less than half of TURN_ANGLE_DEG, the tires are
# probably binding (too much tension on the floor) -- bump TURN_SPEED up by
# IMU_TURN_BOOST_FACTOR to break through the friction. The boosted speed is
# clamped to 1.0. Set IMU_TURN_BOOST_AFTER_S = None to disable the boost.
IMU_TURN_BOOST_AFTER_S = 6     # how long to wait before boosting a slow spin
IMU_TURN_BOOST_FACTOR = 1.4      # multiply TURN_SPEED by this when stalled

# ---------------------------------------------------------------------------
# Motor driver: tank / skid steer, one DIR + one PWM pin per motor (BCM).
#   'dir' sets direction (a single 1/0 line), 'pwm' sets speed (0..100% duty).
#   'invert' flips that side's forward sense in software -- set it True instead
#   of swapping wires if a wheel spins the wrong way.
# ---------------------------------------------------------------------------
MOTORS = {
    "left":  {"dir": 20, "pwm": 13, "invert": False},
    "right": {"dir": 16, "pwm": 12, "invert": False},
}
MOTOR_PWM_HZ = 1000              # PWM frequency on the speed pins
MOTOR_DEADZONE = 0.05            # |side command| below this counts as stop

# ---------------------------------------------------------------------------
# Control loop.
# ---------------------------------------------------------------------------
CONTROL_LOOP_HZ = 20.0           # how often we read sensors and decide

# ---------------------------------------------------------------------------
# Bring-up mode.
#   USE_SENSORS = False -> ignore the ultrasonic sensors entirely and just run
#       the open-loop maneuver script below. Use this to check the motors,
#       H-bridges and turning on the real car (no sensors / dividers needed yet).
#   USE_SENSORS = True  -> normal sensor-driven navigation.
# ---------------------------------------------------------------------------
USE_SENSORS = True

# Open-loop maneuver script used when USE_SENSORS is False.
# Each step is (action, seconds), where action is one of:
#   "forward", "left", "right", "stop".
# Turn steps use TURN_TIME_S so they should be ~90 degrees once it's tuned.
DRIVE_TEST_SEQUENCE = [
    ("forward", 2.0),
    ("left",    TURN_TIME_S),
    ("forward", 2.0),
    ("right",   TURN_TIME_S),
    ("forward", 2.0),
    ("stop",    1.0),
]
