"""
Off-hardware simulation of the IMU + odometry navigation.

Drives the NavigationController through open space (no walls in sensor range) so
the run is entirely IMU/odometry driven: the car cruises down a lane, turns from
odometry at the arena length, then reaches a (demo) pit and disposes. Emulates
main.py by feeding the "true" heading back as the IMU yaw and by calling
complete_turn()/complete_dispose() after those maneuvers. Run with:

    python3 simulate.py

The pit is placed for the demo so the car reaches it after one U-turn.
"""

import types

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


def cfg_demo():
    """Config copy with a pit placed to be reached after one U-turn (see below)."""
    base = {k: getattr(config, k) for k in dir(config) if k.isupper()}
    # After the first right U-turn from the start lane the car is at x ~= START+35,
    # heading 180, driving back down; put the pit there so the demo disposes.
    base.update(PIT_X_CM=config.START_X_CM + config.LANE_WIDTH_CM,
                PIT_Y_CM=130.0)
    return types.SimpleNamespace(**base)


def main():
    cfg = cfg_demo()
    nav = NavigationController(cfg)
    nav.set_origin(0.0)
    heading = 0.0  # the sim's "true" heading, fed back as the IMU yaw each tick
    dt = 0.2

    print(f"arena {cfg.ARENA_WIDTH_CM:.0f}x{cfg.ARENA_LENGTH_CM:.0f}  "
          f"start ({cfg.START_X_CM:.0f},{cfg.START_Y_CM:.0f})  "
          f"pit ({cfg.PIT_X_CM:.0f},{cfg.PIT_Y_CM:.0f})\n")

    for step in range(200):
        cmd = nav.decide(reading(), yaw=heading, dt=dt)
        extra = f" steer={cmd.steer:+.2f}" if cmd.action is Action.FORWARD else ""
        print(f"{step:3d} MODE={nav.mode.name:<9} {nav.pose_str()} "
              f"-> {cmd.action.name:<10}{extra}  [{cmd.reason}]")

        if cmd.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
            nav.complete_turn()          # main.py runs the blocking U-turn here
            heading = nav.target_heading  # the car now physically faces the new lane
        elif cmd.action is Action.DISPOSE:
            nav.complete_dispose()        # main.py runs the dump here
            print("\n--> reached the pit and disposed; sweep would resume.")
            break


if __name__ == "__main__":
    main()
