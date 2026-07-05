import math

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

# How many lanes make up a full sweep of the arena = how wide the arena is
# divided by how far we step each lane (rounded UP so the far edge is covered).
# The car counts lanes as it goes and STOPS once it has finished lane NUM_LANES-1
# (coverage complete). Override the number here if you want to sweep fewer/more
# lanes than the plain division gives (e.g. partial coverage, or extra overlap).
NUM_LANES = math.ceil(ARENA_WIDTH_CM / LANE_WIDTH_CM)   # e.g. 210 / 35 = 6

# Measured forward travel speed at DRIVE_SPEED, in cm/s. NOTE: position is now
# wall-referenced (front sensors), so this is only a FALLBACK/BRIDGE value:
#   * it bridges along-lane position for the brief ticks the wall isn't seen,
#   * it provides the rough "where should the wall be" prior (WALL_EXPECT_TOL_CM),
#   * it times the LANE_WIDTH sideways shift inside the U-turn.
# None of these need high accuracy (they tolerate wheel slip on bumps), but a
# roughly-right value still helps. Measure it once and move on.
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
PIT_ARRIVAL_RADIUS_CM = 30.0        # how close (cm) to the pit centre counts as "arrived"
DISPOSE_BACK_INTO_PIT = True        # True = orient the back toward the pit before reversing
# How far to reverse (after orienting) to place the rear directly over the small
# pit, and how fast. TUNE on the real setup so the rear OVERHANGS the pit but the
# drive wheels stay on the edge (the pit is roughly car-sized). The car pulls the
# same distance forward again after dumping to get clear before resuming.
DISPOSE_REVERSE_CM = 20.0
DISPOSE_REVERSE_SPEED = 0.3         # slow, for precise placement (0..1)
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
#
#   The 3 FRONT sensors are the PRIMARY position reference (distance to the end
#   wall = how far down the lane we are). Mount them:
#     * spread edge-to-edge across the front (left-edge / center / right-edge) and
#       OUTBOARD of the collection bucket, so no sensor stares into its own scoop
#       and the outer pair gives a wide "is it a full wall or just a block?" baseline,
#     * LEVEL and ABOVE block height, aimed straight ahead, so a flat wall reflects
#       cleanly and low blocks/bumps are not seen.
# ---------------------------------------------------------------------------
# NOTE: pins 12, 13, 16, 20 are used by the motors (see MOTORS below), so the
# sensors are wired clear of them. Change these to match your actual wiring.
SENSORS = {
    "front_left":   {"trig": 23, "echo": 24, "enabled": True},  # left edge, outboard of bucket
    "front_center": {"trig": 27,  "echo": 22, "enabled": True},  # centre
    "front_right":  {"trig": 6, "echo": 5, "enabled": True},  # right edge, outboard of bucket
    "right_front":  {"trig": 17, "echo": 4, "enabled": False},  # right, toward front
    "right_rear":   {"trig": 22, "echo": 25, "enabled": False},  # right, toward rear
}

# Logical groupings used by the navigation logic.
FRONT_SENSORS = ("front_left", "front_center", "front_right")
RIGHT_SENSORS = ("right_front", "right_rear")

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
RIGHT_WALL_DISTANCE_CM = 25.0    # closer than this => a wall is present on the right
RIGHT_TARGET_DISTANCE_CM = 18.0  # desired gap to the right wall (only used if USE_WALL_FOLLOW)

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
#   position coasts on odometry for up to BRIDGE_MAX_S until the wall re-appears.
# ---------------------------------------------------------------------------
FRONT_AGREE_TOL_CM = 15.0        # front readings within this of each other "agree"
FRONT_AGREE_MIN_COUNT = 2        # need at least this many agreeing (K of 3)
WALL_EXPECT_TOL_CM = 70.0        # how far odometry may disagree with the wall and still trust it (generous: odometry is rough)
WALL_PERSIST_TICKS = 3           # consecutive ticks the wall-stop must hold before turning
BRIDGE_MAX_S = 2.0               # max time to coast on odometry when the wall drops out

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
