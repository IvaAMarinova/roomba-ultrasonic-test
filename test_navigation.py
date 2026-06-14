"""
Assertions for the turn logic. Runs with plain `python3 test_navigation.py`
(no dependencies) and is also discoverable by pytest.
"""

import config
from navigation import NavigationController, Action

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


def nav():
    return NavigationController(config)


# -- cruising ---------------------------------------------------------------

def test_open_space_cruises_forward():
    cmd = nav().decide(reading())
    assert cmd.action is Action.FORWARD
    assert cmd.speed == config.DRIVE_SPEED


def test_close_front_slows_but_keeps_going():
    cmd = nav().decide(reading(front_center=config.FRONT_SLOW_DISTANCE_CM - 5,
                               right_front=18, right_rear=18))
    assert cmd.action is Action.FORWARD
    assert cmd.speed == config.SLOW_SPEED


def test_too_far_from_wall_trims_right():
    cmd = nav().decide(reading(front_center=150, right_front=40, right_rear=40))
    assert cmd.action is Action.FORWARD
    assert cmd.steer > 0  # positive = toward the wall (right)


def test_nose_angled_away_trims_right():
    # front-right reads farther than rear-right -> nose pointed away from wall.
    cmd = nav().decide(reading(front_center=150, right_front=24, right_rear=16))
    assert cmd.steer > 0


# -- trigger 1: wall straight ahead ----------------------------------------

def test_wall_ahead_with_right_wall_turns_left():
    # Inside corner: blocked ahead and a wall on the right -> turn away (left).
    cmd = nav().decide(reading(front_left=18, front_center=15, front_right=19,
                               right_front=16, right_rear=16))
    assert cmd.action is Action.TURN_LEFT


def test_wall_ahead_with_open_right_turns_right():
    # Blocked ahead but open on the right -> turn into the opening (right).
    cmd = nav().decide(reading(front_center=15, right_front=80, right_rear=80))
    assert cmd.action is Action.TURN_RIGHT


def test_any_front_sensor_triggers_stop_distance():
    # Only the side front sensor sees the wall; nearest-of-three still triggers.
    cmd = nav().decide(reading(front_left=15, right_front=16, right_rear=16))
    assert cmd.action is Action.TURN_LEFT


# -- right side never triggers a turn (no obstacles in a bare rectangle) ----

def test_right_wall_present_still_cruises():
    # A wall on the right is normal lane-following, never a turn on its own.
    cmd = nav().decide(reading(front_center=150, right_front=18, right_rear=18))
    assert cmd.action is Action.FORWARD


def test_right_wall_dropping_away_still_cruises():
    # Even a sharp right-side discontinuity must not turn the car: in a closed
    # rectangle this can't be a corner, and we have no obstacles.
    cmd = nav().decide(reading(front_center=150, right_front=120, right_rear=16))
    assert cmd.action is Action.FORWARD


def test_fully_open_right_cruises():
    cmd = nav().decide(reading(front_center=150, right_front=INF, right_rear=INF))
    assert cmd.action is Action.FORWARD


# -- route stub: serpentine alternation ------------------------------------

def test_serpentine_turn_alternates_with_open_right():
    n = nav()
    # Right side clear (no wall to veto) -> turn direction follows the schedule
    # and must alternate at each end wall to walk the sweep across the arena.
    clear = config.RIGHT_WALL_DISTANCE_CM + 10.0
    first = n.decide(reading(front_center=15, right_front=clear, right_rear=clear))
    second = n.decide(reading(front_center=15, right_front=clear, right_rear=clear))
    assert first.action is not second.action
    assert {first.action, second.action} == {Action.TURN_LEFT, Action.TURN_RIGHT}


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
