# ---------------------------------------------------------------------------
# Arena geometry (known and fixed for the competition).
# Not used by the turn logic directly yet, but kept here so the future route
# planner has a single source of truth.
# ---------------------------------------------------------------------------
ARENA_WIDTH_CM = 200.0
ARENA_LENGTH_CM = 200.0
LANE_WIDTH_CM = 30.0          # width of one serpentine sweep lane

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
    "right_front":  {"trig": 17, "echo": 27, "enabled": False},  # right, toward front
    "right_rear":   {"trig": 22, "echo": 25, "enabled": False},  # right, toward rear
}

# Logical groupings used by the navigation logic.
FRONT_SENSORS = ("front_left", "front_center", "front_right")
RIGHT_SENSORS = ("right_front", "right_rear")

# ---------------------------------------------------------------------------
# Decision thresholds (centimetres).
# ---------------------------------------------------------------------------
FRONT_STOP_DISTANCE_CM = 20.0    # wall straight ahead -> end of lane, must turn
FRONT_SLOW_DISTANCE_CM = 40.0    # start slowing down / preparing to turn
RIGHT_WALL_DISTANCE_CM = 25.0    # closer than this => a wall is present on the right
RIGHT_TARGET_DISTANCE_CM = 18.0  # desired gap to the right wall while wall-following

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
SLOW_SPEED = 0.3                 # forward speed when an obstacle is getting close
TURN_SPEED = 0.5                 # in-place rotation speed
STEER_CORRECTION_GAIN = 0.015    # how hard to trim heading against the right wall
MAX_STEER_TRIM = 0.4             # clamp on the wall-follow steering trim

# Seconds to rotate 90 degrees in place at TURN_SPEED. No IMU/encoders: this is
# just a tuned constant -- measure it on the actual car and adjust.
TURN_TIME_S = 0.6

# ---------------------------------------------------------------------------
# Motor driver: tank / skid steer, one DIR + one PWM pin per motor (BCM).
#   'dir' sets direction (a single 1/0 line), 'pwm' sets speed (0..100% duty).
#   'invert' flips that side's forward sense in software -- set it True instead
#   of swapping wires if a wheel spins the wrong way.
# ---------------------------------------------------------------------------
MOTORS = {
    "left":  {"dir": 16, "pwm": 12, "invert": False},
    "right": {"dir": 20, "pwm": 13, "invert": False},
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
USE_SENSORS = False

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
