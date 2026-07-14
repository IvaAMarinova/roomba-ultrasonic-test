"""
Off-hardware simulation of the wall-referenced navigation.

Unlike a pure "open space" run, this tracks a *true* position and synthesizes the
front-sensor readings from it (gap to the end wall = ARENA_LENGTH - true_y), so the
controller actually localizes off the wall. It emulates main.py by feeding the true
heading back as the IMU yaw and calling complete_turn()/complete_dispose() after
those maneuvers. Run with:

    python3 simulate.py

You should see src=WALL while a wall is in range, the turn firing at the wall
standoff (not a timer), and disposal at the pit.
"""

import types

import config
from navigation import NavigationController, Action

INF = float("inf")


def reading(front):
    """Three front sensors all seeing `front` (INF if the wall is out of range)."""
    return {
        "front_left": front, "front_center": front, "front_right": front,
        "back_left": INF, "back_right": INF,
    }


def cfg_demo():
    """Config copy with a pit placed to be reached on the 2nd lane (x=+LANE_WIDTH)."""
    base = {k: getattr(config, k) for k in dir(config) if k.isupper()}
    base.update(PIT_X_CM=config.START_X_CM + config.LANE_WIDTH_CM, PIT_Y_CM=130.0)
    return types.SimpleNamespace(**base)


def main():
    cfg = cfg_demo()
    nav = NavigationController(cfg)
    nav.set_origin(0.0)

    true_y = 0.0
    forward = True          # +y on even lanes, -y on odd lanes
    heading = 0.0
    dt = 0.2

    print(f"arena {cfg.ARENA_WIDTH_CM:.0f}x{cfg.ARENA_LENGTH_CM:.0f}  "
          f"start ({cfg.START_X_CM:.0f},{cfg.START_Y_CM:.0f})  "
          f"pit ({cfg.PIT_X_CM:.0f},{cfg.PIT_Y_CM:.0f})\n")

    for step in range(1000):
        gap = (cfg.ARENA_LENGTH_CM - true_y) if forward else true_y
        front = gap if gap <= cfg.SENSOR_MAX_RANGE_CM else INF
        cmd = nav.decide(reading(front), yaw=heading, dt=dt)

        extra = (f" steer={cmd.steer:+.2f}"
                 if cmd.action in (Action.FORWARD, Action.REVERSE) else "")
        print(f"{step:3d} MODE={nav.mode.name:<9} {nav.pose_str()} "
              f"true_y={true_y:5.1f} -> {cmd.action.name:<10}{extra}  [{cmd.reason}]")

        if cmd.action is Action.FORWARD:
            step_cm = cfg.DRIVE_CM_PER_S * (cmd.speed / cfg.DRIVE_SPEED) * dt
            true_y += step_cm if forward else -step_cm
        elif cmd.action is Action.REVERSE:
            step_cm = cfg.DRIVE_CM_PER_S * (cmd.speed / cfg.DRIVE_SPEED) * dt
            true_y += -step_cm if forward else step_cm
        elif cmd.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
            nav.complete_turn()
            heading = nav.target_heading
            forward = not forward          # reversed down the next lane
        elif cmd.action is Action.DISPOSE:
            print("    --> at the pit: dispose out the back, then keep sweeping")
            nav.complete_dispose()         # main.py runs the reverse-and-dump here
        elif cmd.action is Action.ALIGN_CENTER:
            print("    --> align at wall, then face departure heading")
            if cmd.face_heading is not None:
                nav.complete_align_center(cmd.face_heading)
        elif cmd.action is Action.ALIGN_PIT:
            print("    --> align over pit with side sensors, then dump")
            nav._pit_handled = True
            nav.complete_dispose()
        elif cmd.action is Action.STOP:
            print("\n--> coverage complete; car stops.")
            break


if __name__ == "__main__":
    main()
