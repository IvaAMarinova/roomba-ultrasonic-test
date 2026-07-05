# ---------------------------------------------------------------------------
# Arena geometry (known and fixed for the competition).
# ---------------------------------------------------------------------------
ARENA_WIDTH_CM = 210.0        # across the lanes (the car steps sideways here)
ARENA_LENGTH_CM = 300.0       # along each lane (the long axis the car runs)

ROBOT_WIDTH_CM = 50.0         # physical width of the car

# How far the car shifts sideways at each end-of-lane U-turn. For full coverage
# with no gaps this must be <= ROBOT_WIDTH_CM; a little less gives overlap.
# 210 / 45 ~= 5 lanes. Bigger = wider shift (faster, but leaves unswept strips
# once it exceeds the car width).
LANE_WIDTH_CM = 35.0

# Measured forward travel speed at DRIVE_SPEED, in cm/s. Used both to convert the
# lane-width shift into an open-loop drive time AND for dead-reckoning odometry
# (distance-along-lane = DRIVE_CM_PER_S * time). MEASURE THIS ACCURATELY on the
# real car (drive forward at DRIVE_SPEED for a known time, divide distance by
# time) -- odometry-based turning and pit arrival both depend on it.
DRIVE_CM_PER_S = 30.0

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
# Disposal pit (known fixed location, in the start-relative frame above).
#   The car sweeps the arena and, when its pose comes within
#   PIT_ARRIVAL_RADIUS_CM of (PIT_X_CM, PIT_Y_CM), it enters DISPOSING: it
#   rotates so its BACK faces the pit (waste-truck style) and dumps.
#   SET THESE to the real pit location for the competition arena.
# ---------------------------------------------------------------------------
PIT_X_CM = ARENA_WIDTH_CM / 2.0     # TODO: real pit centre X (placeholder: arena middle)
PIT_Y_CM = ARENA_LENGTH_CM          # TODO: real pit centre Y (placeholder: far wall)
PIT_ARRIVAL_RADIUS_CM = 30.0        # how close (cm) to the pit centre counts as "arrived"
DISPOSE_BACK_INTO_PIT = True        # True = orient back toward the pit before dumping
DISPOSE_HOLD_S = 2.0                # placeholder dwell while "dumping" (until servo lands)

# ---------------------------------------------------------------------------
# Collection (future servo).
#   The collection servo will report how many blocks are in the bucket; when it
#   reaches COLLECTION_CAPACITY_BLOCKS the bucket is "full". For now the count is
#   a stub (see actuators.Collector) and disposal triggers purely on reaching the
#   pit -- wiring is a one-line change once the servo exists.
# ---------------------------------------------------------------------------
COLLECTION_CAPACITY_BLOCKS = 10     # TODO: real bucket capacity; count comes from the collection servo later

# ---------------------------------------------------------------------------
# Ultrasonic sensor layout.
#   5x HC-SR04: 3 facing forward, 2 facing right.
#   Each entry: logical name -> {TRIG pin, ECHO pin, enabled} in BCM numbering.
#   Adjust the pin numbers to match your wiring.
#   "enabled": False -> that sensor is never read (its GPIO is left untouched)
#   and its distance always reports as "no echo". Use it to bring sensors up one
#   at a time, or to ignore one that isn't wired / is faulty.
# ---------------------------------------------------------------------------
# NOTE: pins 12, 13, 16, 20 are used by the motors (see MOTORS below), so the
# sensors are wired clear of them. Change these to match your actual wiring.
SENSORS = {
    "front_left":   {"trig": 23, "echo": 24, "enabled": True},
    "front_center": {"trig": 27,  "echo": 22, "enabled": True},
    "front_right":  {"trig": 6, "echo": 5, "enabled": True},
    "right_front":  {"trig": 17, "echo": 4, "enabled": False},  # right, toward front
    "right_rear":   {"trig": 22, "echo": 25, "enabled": False},  # right, toward rear
}

# Logical groupings used by the navigation logic.
FRONT_SENSORS = ("front_left", "front_center", "front_right")
RIGHT_SENSORS = ("right_front", "right_rear")

# ---------------------------------------------------------------------------
# Decision thresholds (centimetres).
#   End-of-lane is now decided by ODOMETRY (distance driven vs ARENA_LENGTH_CM,
#   see LANE_END_MARGIN_CM below). The FRONT_* ultrasonic thresholds are the
#   SAFETY FALLBACK: if a wall shows up closer than expected (odometry drift, an
#   unexpected obstacle) they force the turn early so we never drive into it.
# ---------------------------------------------------------------------------
FRONT_STOP_DISTANCE_CM = 40.0    # FALLBACK: wall this close ahead -> turn now, even if odometry disagrees
FRONT_SLOW_DISTANCE_CM = 50.0    # start slowing down / preparing to turn
# NOTE: STOP must be < SLOW, and both are clearances (small), NOT sensor range.
RIGHT_WALL_DISTANCE_CM = 25.0    # closer than this => a wall is present on the right
RIGHT_TARGET_DISTANCE_CM = 18.0  # desired gap to the right wall (only used if USE_WALL_FOLLOW)

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
DRIVE_SPEED = 0.6                # nominal forward speed (0..1)
SLOW_SPEED = 0.4                 # forward speed when an obstacle is getting close
TURN_SPEED = 0.2                 # in-place rotation speed
STEER_CORRECTION_GAIN = 0.015    # how hard to trim heading against the right wall (wall-follow only)
MAX_STEER_TRIM = 0.4             # clamp on the wall-follow steering trim

# ---------------------------------------------------------------------------
# Straight-line driving: IMU heading-hold (primary) vs right-wall follow (legacy).
#   While DRIVING, the car cruises forward and trims its steering to hold the
#   lane's target heading using the IMU, instead of following the right wall.
#   HEADING_HOLD_GAIN turns a heading error (deg) into a steer trim; the result
#   is clamped to +/-MAX_HEADING_TRIM. Positive steer = toward the car's right.
# ---------------------------------------------------------------------------
USE_WALL_FOLLOW = False           # False = IMU heading-hold (default), True = legacy right-wall trim
HEADING_HOLD_GAIN = 0.02          # steer trim per degree of heading error
MAX_HEADING_TRIM = 0.4            # clamp on the heading-hold steering trim

# ---------------------------------------------------------------------------
# Odometry-based end-of-lane.
#   Distance driven down the current lane is integrated from DRIVE_CM_PER_S; once
#   it reaches ARENA_LENGTH_CM - LANE_END_MARGIN_CM the car turns (the ultrasonic
#   front stop is only a fallback for when a wall appears sooner than expected).
# ---------------------------------------------------------------------------
LANE_END_MARGIN_CM = 40.0         # turn this far before the far wall (odometry trigger)

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
    "left":  {"dir": 16, "pwm": 12, "invert": True},
    "right": {"dir": 20, "pwm": 13, "invert": True},
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
