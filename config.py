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
#   Each entry: logical name -> (TRIG pin, ECHO pin) in BCM numbering.
#   Adjust the pin numbers to match your wiring.
# ---------------------------------------------------------------------------
SENSORS = {
    "front_left":   {"trig": 5,  "echo": 6},
    "front_center": {"trig": 13, "echo": 19},
    "front_right":  {"trig": 26, "echo": 21},
    "right_front":  {"trig": 16, "echo": 20},   # right side, toward the front
    "right_rear":   {"trig": 12, "echo": 25},   # right side, toward the rear
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
# Motor driver: two H-bridges, tank / skid steer.
#   Each side has two direction pins (IN1, IN2) in BCM numbering:
#     forward = (IN1=1, IN2=0), reverse = (IN1=0, IN2=1), stop = (0, 0).
#   'en' is the H-bridge enable pin:
#     - set it to a pin number to control speed with PWM, or
#     - set it to None if the enable line is tied high and you only drive
#       direction (full-speed on/off). In that mode speed magnitude is treated
#       as on/off against MOTOR_DEADZONE, so steering still works bang-bang.
# ---------------------------------------------------------------------------
MOTORS = {
    "left":  {"in1": 17, "in2": 27, "en": 22},
    "right": {"in1": 23, "in2": 24, "en": 18},
}
MOTOR_PWM_HZ = 1000              # PWM frequency on the enable pins (if used)
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
