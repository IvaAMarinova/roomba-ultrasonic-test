"""
Assertions for the IMU + odometry navigation logic. Runs with plain
`python3 test_navigation.py` (no dependencies) and is also discoverable by pytest.

The controller now decides from three inputs -- ultrasonic readings, IMU yaw, and
elapsed dt -- so the helpers below default yaw/dt to a stationary tick (dt=0, so
no odometry accumulates) unless a test drives time forward on purpose.
"""

import types

import config
from navigation import NavigationController, Action, Mode

INF = float("inf")


def reading(front_left=INF, front_center=INF, front_right=INF,
            right_front=INF, right_rear=INF):
    return {
        "front_left": front_left,
        "front_center": front_center,
        "front_right": front_right,
        "right_front": right_front,
        "right_rear": right_rear,
    }


def front_wall(dist, **kw):
    """All three front sensors agree on `dist` -- a real wall ahead."""
    return reading(front_left=dist, front_center=dist, front_right=dist, **kw)


def cfg_with(**overrides):
    """A copy of config with a few values overridden (for isolated tests)."""
    base = {k: getattr(config, k) for k in dir(config) if k.isupper()}
    base.update(overrides)
    return types.SimpleNamespace(**base)


def nav(cfg=config):
    n = NavigationController(cfg)
    n.set_origin(0.0)  # IMU present, current facing = heading 0
    return n


# -- cruising ---------------------------------------------------------------

def test_open_space_cruises_forward():
    cmd = nav().decide(reading(), yaw=0.0, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert cmd.speed == config.DRIVE_SPEED


def test_on_heading_cruises_straight():
    # Heading matches the lane target (0) -> no steering trim.
    cmd = nav().decide(reading(front_center=150), yaw=0.0, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert abs(cmd.steer) < 1e-9


def test_close_front_slows_but_keeps_going():
    cmd = nav().decide(front_wall(config.FRONT_SLOW_DISTANCE_CM - 5),
                       yaw=0.0, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert cmd.speed == config.SLOW_SPEED


# -- IMU heading-hold trim --------------------------------------------------

def test_heading_right_of_target_trims_left():
    # Nose turned +10deg (to the right) -> steer left (negative) to come back.
    cmd = nav().decide(reading(front_center=150), yaw=10.0, dt=0.0)
    assert cmd.steer < 0


def test_heading_left_of_target_trims_right():
    cmd = nav().decide(reading(front_center=150), yaw=-10.0, dt=0.0)
    assert cmd.steer > 0


def test_no_imu_drives_open_loop_straight():
    n = NavigationController(config)
    n.set_origin(None)  # no IMU heading available
    cmd = n.decide(reading(front_center=150), yaw=None, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert cmd.steer == 0.0


# -- trigger: odometry end-of-lane (no sensor input at all) -----------------

def test_odometry_turns_at_end_of_lane():
    # Pit parked far away so only the odometry end-of-lane trigger is exercised.
    n = nav(cfg_with(PIT_X_CM=-10000.0, PIT_Y_CM=-10000.0))
    cmd = None
    # Drive forward in open space; distance accumulates until the lane is done.
    for _ in range(40):
        cmd = n.decide(reading(), yaw=0.0, dt=1.0)
        if cmd.action is not Action.FORWARD:
            break
    assert cmd.action is Action.TURN_RIGHT
    assert "odometry" in cmd.reason
    assert n.lane_distance >= n.cfg.ARENA_LENGTH_CM - n.cfg.LANE_END_MARGIN_CM


# -- fallback trigger: wall straight ahead ----------------------------------

def test_front_wall_fallback_turns_right_first():
    # From the bottom-left corner the serpentine starts RIGHT; a close front wall
    # forces the turn.
    cmd = nav().decide(front_wall(15), yaw=0.0, dt=0.0)
    assert cmd.action is Action.TURN_RIGHT
    assert "FALLBACK" in cmd.reason


def test_single_drifting_front_sensor_is_ignored():
    stop = config.FRONT_STOP_DISTANCE_CM
    cmd = nav().decide(reading(front_left=stop - 10, front_center=200,
                               front_right=200), yaw=0.0, dt=0.0)
    assert cmd.action is Action.FORWARD


def test_single_dropout_does_not_hide_a_real_wall():
    stop = config.FRONT_STOP_DISTANCE_CM
    cmd = nav().decide(reading(front_left=stop - 10, front_center=stop - 8,
                               front_right=INF), yaw=0.0, dt=0.0)
    assert cmd.action is Action.TURN_RIGHT


def test_serpentine_turns_alternate_right_first():
    n = nav()
    turns = [n.decide(front_wall(15), yaw=0.0, dt=0.0).action for _ in range(4)]
    assert turns == [Action.TURN_RIGHT, Action.TURN_LEFT,
                     Action.TURN_RIGHT, Action.TURN_LEFT]


# -- disposal: pit arrival --------------------------------------------------

def test_pit_arrival_triggers_dispose():
    n = nav()
    n.x, n.y = config.PIT_X_CM, config.PIT_Y_CM  # place the car at the pit
    cmd = n.decide(reading(), yaw=0.0, dt=0.0)
    assert cmd.action is Action.DISPOSE
    assert n.mode is Mode.DISPOSING


def test_dispose_does_not_retrigger_until_leaving_pit():
    n = nav()
    n.x, n.y = config.PIT_X_CM, config.PIT_Y_CM
    assert n.decide(reading(), yaw=0.0, dt=0.0).action is Action.DISPOSE
    n.complete_dispose()  # main.py would run the dump, then this
    # Still inside the pit radius -> must NOT dispose again immediately.
    assert n.decide(reading(), yaw=0.0, dt=0.0).action is not Action.DISPOSE


# -- legacy wall-follow (only when explicitly enabled) ----------------------

def test_wall_follow_trims_toward_far_wall_when_enabled():
    n = nav(cfg_with(USE_WALL_FOLLOW=True))
    cmd = n.decide(reading(front_center=150, right_front=40, right_rear=40),
                   yaw=0.0, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert cmd.steer > 0  # too far from the right wall -> steer toward it


def _run():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failures += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return failures


if __name__ == "__main__":
    raise SystemExit(1 if _run() else 0)
