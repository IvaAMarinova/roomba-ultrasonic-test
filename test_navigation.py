"""
Assertions for the wall-referenced navigation logic. Runs with plain
`python3 test_navigation.py` (no dependencies) and is also discoverable by pytest.

The controller decides from ultrasonic readings + IMU yaw + elapsed dt. Position
down the lane comes from the front wall (with a K-of-N agreement + "near where a
wall is expected" gate), falling back to odometry (DRIVE_CM_PER_S x time) only
when no wall is seen. Helpers default yaw/dt to a stationary tick unless a test
drives time forward on purpose.
"""

import types

import config
from navigation import NavigationController, Action, Mode

INF = float("inf")


def reading(front_left=INF, front_center=INF, front_right=INF,
            back_left=INF, back_right=INF):
    return {
        "front_left": front_left,
        "front_center": front_center,
        "front_right": front_right,
        "back_left": back_left,
        "back_right": back_right,
    }


def front_wall(dist, **kw):
    """All three front sensors agree on `dist` -- a full-width wall ahead."""
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


def drive_to_near_end(n, cfg=config):
    """Place the rover near the far end of the lane (for wall-detection tests)."""
    target = cfg.ARENA_LENGTH_CM - cfg.FRONT_STOP_DISTANCE_CM - 20.0
    n.lane_distance = target
    n.decide(reading(), yaw=0.0, dt=0.0)   # refresh y from lane_distance
    return n


# -- cruising ---------------------------------------------------------------

def test_open_space_cruises_forward():
    cmd = nav().decide(reading(), yaw=0.0, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert cmd.speed == config.DRIVE_SPEED


def test_on_heading_cruises_straight():
    cmd = nav().decide(reading(), yaw=0.0, dt=0.0)
    assert abs(cmd.steer) < 1e-9


def test_heading_right_of_target_trims_left():
    # Heading right of target -> steer left (negative trim).
    cmd = nav().decide(reading(), yaw=10.0, dt=0.0)
    assert cmd.steer > 0   # positive steer = toward car's right; corrects leftward drift


def test_heading_left_of_target_trims_right():
    cmd = nav().decide(reading(), yaw=-10.0, dt=0.0)
    assert cmd.steer < 0


def test_no_imu_drives_open_loop_straight():
    n = NavigationController(config)
    n.set_origin(None)
    cmd = n.decide(reading(), yaw=None, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert cmd.steer == 0.0


# -- wall detection: position source ----------------------------------------

def test_agreeing_wall_near_expected_is_believed():
    # Near the lane end, a full-width wall at the expected gap -> WALL source.
    n = drive_to_near_end(nav())
    n.decide(front_wall(config.FRONT_STOP_DISTANCE_CM + 10), yaw=0.0, dt=0.2)
    assert n.pos_source == "WALL"
    assert n.front_agree == 3


def test_wall_ends_lane_after_persistence():
    n = drive_to_near_end(nav(cfg_with(PIT_X_CM=-1e4, PIT_Y_CM=-1e4)))
    cmd = None
    for _ in range(config.WALL_PERSIST_TICKS):
        cmd = n.decide(front_wall(config.FRONT_STOP_DISTANCE_CM - 10),
                       yaw=0.0, dt=0.2)
    assert cmd.action is Action.TURN_RIGHT       # bottom-left start turns right first
    assert "wall" in cmd.reason


def test_one_tick_wall_does_not_turn_yet():
    # A single close-wall tick must not turn (persistence not met).
    n = drive_to_near_end(nav())
    cmd = n.decide(front_wall(config.FRONT_STOP_DISTANCE_CM - 10), yaw=0.0, dt=0.2)
    if config.WALL_PERSIST_TICKS > 1:
        assert cmd.action is Action.FORWARD


# -- block / bump rejection -------------------------------------------------

def test_single_sensor_object_is_ignored():
    # One sensor sees something close, the other two see nothing -> not a wall.
    n = drive_to_near_end(nav())
    cmd = n.decide(reading(front_left=20), yaw=0.0, dt=0.2)
    assert cmd.action is Action.FORWARD
    assert n.pos_source == "BRIDGE"


def test_agreeing_object_far_from_expected_is_not_the_wall():
    # At the START of a lane, a full-width object 30 cm ahead can't be the end
    # wall (odometry expects it ~3 m away) -> treated as a block, no turn.
    n = nav()
    cmd = n.decide(front_wall(30), yaw=0.0, dt=0.0)
    assert cmd.action is Action.FORWARD
    assert n.pos_source == "BRIDGE"


# -- bridge: wall drops out -------------------------------------------------

def test_wall_dropout_bridges_on_odometry():
    n = drive_to_near_end(nav())
    n.decide(front_wall(config.FRONT_STOP_DISTANCE_CM + 15), yaw=0.0, dt=0.2)
    assert n.pos_source == "WALL"
    y_before = n.y
    cmd = n.decide(reading(), yaw=0.0, dt=0.2)   # wall vanishes
    assert n.pos_source == "BRIDGE"
    assert cmd.action is Action.FORWARD
    assert n.y >= y_before                        # position kept advancing, not lost


# -- odometry backstop (no wall ever seen) ----------------------------------

def test_odometry_backstop_turns_without_any_wall():
    n = nav(cfg_with(PIT_X_CM=-1e4, PIT_Y_CM=-1e4))
    cmd = None
    for _ in range(60):
        cmd = n.decide(reading(), yaw=0.0, dt=1.0)   # never any wall
        if cmd.action is not Action.FORWARD:
            break
    assert cmd.action is Action.TURN_RIGHT
    assert "backstop" in cmd.reason


def test_serpentine_turns_alternate_right_first():
    n = nav(cfg_with(PIT_X_CM=-1e4, PIT_Y_CM=-1e4))
    turns = []
    for _ in range(200):
        c = n.decide(reading(), yaw=0.0, dt=1.0)
        if c.action is not Action.FORWARD:
            turns.append(c.action)
            if len(turns) == 4:
                break
    assert turns == [Action.TURN_RIGHT, Action.TURN_LEFT,
                     Action.TURN_RIGHT, Action.TURN_LEFT]


# -- coverage / done condition ----------------------------------------------

def test_stops_when_all_lanes_swept():
    # Small arena so it finishes fast; no wall ever, so odometry backstop turns.
    cfg = cfg_with(ARENA_WIDTH_CM=70.0, LANE_WIDTH_CM=35.0, NUM_LANES=2,
                   PIT_X_CM=-1e4, PIT_Y_CM=-1e4)
    n = nav(cfg)
    cmd = None
    for _ in range(400):
        cmd = n.decide(reading(), yaw=n.target_heading, dt=1.0)
        if cmd.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
            n.complete_turn()          # emulate main executing the U-turn
        elif cmd.action is Action.STOP:
            break
    assert cmd.action is Action.STOP
    assert n.mode is Mode.DONE
    assert "coverage complete" in cmd.reason


def test_done_latches_stopped():
    cfg = cfg_with(ARENA_WIDTH_CM=70.0, LANE_WIDTH_CM=35.0, NUM_LANES=2,
                   PIT_X_CM=-1e4, PIT_Y_CM=-1e4)
    n = nav(cfg)
    for _ in range(400):
        c = n.decide(reading(), yaw=n.target_heading, dt=1.0)
        if c.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
            n.complete_turn()
        elif c.action is Action.STOP:
            break
    # Once done, further ticks keep returning STOP (never resumes sweeping).
    assert n.decide(reading(), yaw=n.target_heading, dt=1.0).action is Action.STOP
    assert n.decide(front_wall(20), yaw=n.target_heading, dt=1.0).action is Action.STOP


# -- cross-lane position from lane counting ---------------------------------

def test_lane_index_sets_cross_lane_x():
    n = nav()
    assert n.x == config.START_X_CM
    n.complete_turn()                        # steps one lane sideways (persistent x)
    assert abs(n.x - (config.START_X_CM + config.LANE_WIDTH_CM)) < 1e-9
    n.decide(reading(), yaw=0.0, dt=0.0)     # x is lane-counted only -> unchanged by a tick
    assert abs(n.x - (config.START_X_CM + config.LANE_WIDTH_CM)) < 1e-9


# -- disposal: pit arrival --------------------------------------------------

def test_pit_arrival_triggers_dispose():
    n = nav(cfg_with(PIT_X_CM=config.START_X_CM, PIT_Y_CM=60.0,
                     PIT_ARRIVAL_RADIUS_CM=15.0))
    cmd = None
    for _ in range(20):
        cmd = n.decide(reading(), yaw=0.0, dt=0.5)   # odometry up the lane
        if cmd.action is Action.DISPOSE:
            break
    assert cmd.action is Action.DISPOSE
    assert n.mode is Mode.DISPOSING


def test_dispose_does_not_retrigger_until_leaving_pit():
    n = nav(cfg_with(PIT_X_CM=config.START_X_CM, PIT_Y_CM=60.0,
                     PIT_ARRIVAL_RADIUS_CM=15.0))
    for _ in range(20):
        if n.decide(reading(), yaw=0.0, dt=0.5).action is Action.DISPOSE:
            break
    n.complete_dispose()
    # Still inside the pit radius (barely moved) -> must NOT dispose again.
    assert n.decide(reading(), yaw=0.0, dt=0.0).action is not Action.DISPOSE


def test_complete_turn_preserves_y_at_start_wall():
    """U-turn onto a +y lane must not snap y to 0 (false pit arrival)."""
    n = nav()
    n._lane_index = 1
    n.x = 35.0
    n.y = 16.8
    n.target_heading = -180.0
    n.lane_distance = 143.2
    n.complete_turn()
    assert abs(n.lane_distance - 16.8) < 0.1
    n.decide(reading(), yaw=0.0, dt=0.0)
    assert abs(n.y - 16.8) < 0.1


def test_dispose_face_heading_on_start_wall_pit():
    n = nav(cfg_with(PIT_X_CM=75.0, PIT_Y_CM=0.0))
    n.x, n.y = 70.0, 0.0   # dy=0 would break bearing_to_pit + 180
    assert n.dispose_face_heading() == 0.0


def test_end_wall_uses_min_when_sensors_agree_within_tol():
    n = nav()
    dist, agree = n.end_wall_ahead(reading(front_left=50, front_right=55))
    assert agree == 2
    assert 50 <= dist <= 55


def test_lane_end_ignores_disagreeing_sensors():
    n = nav()
    dist, agree = n.end_wall_ahead(reading(front_left=30, front_right=150))
    assert dist == INF
    assert agree == 2


def test_lane_end_rejects_close_reading_far_from_expected():
    """Side obstacle at 30 cm must not end the lane when odometry says y~30 of 212."""
    n = nav()
    n.lane_distance = 30.0
    cmd = None
    for _ in range(config.WALL_PERSIST_TICKS):
        cmd = n.decide(reading(front_left=29, front_right=28), yaw=0.0, dt=0.2)
    assert cmd.action is Action.FORWARD


def test_phantom_far_wall_not_anchored_near_start():
  n = nav()
  n.lane_distance = 10.0
  n.decide(reading(front_left=179, front_right=179), yaw=0.0, dt=0.0)
  assert n.pos_source == "BRIDGE"
  assert n.lane_distance == 10.0


def test_lane_distance_never_negative():
    n = nav()
    n.lane_distance = -5.0
    n.decide(reading(), yaw=0.0, dt=0.0)
    assert n.lane_distance >= 0.0


# -- post-dispose / blocking-gap regressions (from 2026-07 run logs) --------

def test_large_dt_does_not_bridge_odometry():
    """Blocking dispose/scoop gaps must not integrate seconds of forward motion."""
    n = nav()
    n.lane_distance = 27.5
    n._last_action = Action.FORWARD
    n._last_speed = config.DRIVE_SPEED
    n.decide(reading(), yaw=0.0, dt=5.0)
    assert n.lane_distance < 40.0


def test_note_blocking_clears_odometry_state():
    n = nav()
    n.lane_distance = 27.5
    n._last_action = Action.FORWARD
    n._last_speed = config.DRIVE_SPEED
    n.note_blocking_maneuver()
    before = n.lane_distance
    n.decide(reading(), yaw=0.0, dt=2.0)
    assert n.lane_distance == before


def test_complete_dispose_clears_odometry_bridge():
    """After dispose, a long dt tick must not jump y from pit (~27) to far wall."""
    n = nav(cfg_with(PIT_X_CM=75.0, PIT_Y_CM=0.0, PIT_ARRIVAL_RADIUS_CM=50.0))
    n._lane_index = 2
    n.x, n.y = 70.0, 27.5
    n.lane_distance = 27.5
    n.target_heading = 0.0
    n._last_action = Action.FORWARD
    n._last_speed = config.DRIVE_SPEED
    n.complete_dispose()
    n.decide(reading(front_left=11.0, front_right=8.0), yaw=10.0, dt=2.6)
    assert n.lane_distance < 80.0
    assert abs(n.y - 27.5) < 55.0


def test_wall_not_anchored_when_heading_misaligned():
    """Inflated odometry + close front reading while sideways must not snap to y=202."""
    n = nav()
    n.lane_distance = 147.0
    n.heading_rel = -79.0
    n.target_heading = 0.0
    n.decide(reading(front_left=10.0, front_right=8.0), yaw=-79.0, dt=0.0)
    assert n.pos_source == "BRIDGE"
    assert abs(n.lane_distance - 147.0) < 5.0


def test_lane_end_rejected_when_heading_misaligned():
    """Post-dispose run: wall at 13 cm while heading ~-77 must not trigger U-turn."""
    n = nav(cfg_with(PIT_X_CM=-1e4, PIT_Y_CM=-1e4))
    n.lane_distance = 199.0
    n.heading_rel = -77.0
    n.target_heading = 0.0
    cmd = None
    for _ in range(config.WALL_PERSIST_TICKS):
        cmd = n.decide(reading(front_left=13.0, front_right=13.0),
                       yaw=-77.0, dt=0.2)
    assert cmd.action is Action.FORWARD


def test_pit_y27_close_sensors_do_not_end_lane():
    """Lane 2 at pit: disagreeing side readings must not end the lane early."""
    n = nav()
    n.lane_distance = 27.5
    cmd = None
    for _ in range(config.WALL_PERSIST_TICKS):
        cmd = n.decide(reading(front_left=29.0, front_right=28.0), yaw=0.0, dt=0.2)
    assert cmd.action is Action.FORWARD


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
