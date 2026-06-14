"""
Off-hardware simulation of the turn logic.

Feeds the NavigationController a scripted sequence of sensor readings that walks
the car down a lane, into a wall straight ahead, and past a convex corner on its
right, printing the decision at every step. Run with:

    python3 simulate.py

This is the quickest way to eyeball that the two turn triggers fire correctly.
"""

import config
from navigation import NavigationController

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


# A scripted run. Each entry is (label, readings).
SCENARIO = [
    ("cruise, tracking right wall",
     reading(front_center=150, right_front=18, right_rear=18)),

    ("drifting away from right wall (should trim right)",
     reading(front_center=120, right_front=30, right_rear=26)),

    ("nose angled toward wall (should trim left)",
     reading(front_center=100, right_front=12, right_rear=20)),

    ("wall getting close ahead (should slow)",
     reading(front_center=35, right_front=18, right_rear=18)),

    ("WALL STRAIGHT AHEAD (first end-of-lane turn -> LEFT)",
     reading(front_left=18, front_center=15, front_right=19,
             right_front=16, right_rear=16)),

    ("cruise along the next lane",
     reading(front_center=150, right_front=18, right_rear=18)),

    ("WALL STRAIGHT AHEAD again (alternates -> RIGHT)",
     reading(front_center=15, right_front=18, right_rear=18)),

    ("right wall drops away (NOT a turn in a bare rectangle -> keep cruising)",
     reading(front_center=150, right_front=120, right_rear=18)),

    ("open arena, no walls in range (cruise straight)",
     reading()),
]


def main():
    nav = NavigationController(config)
    for label, readings in SCENARIO:
        cmd = nav.decide(readings)
        extra = f" steer={cmd.steer:+.2f}" if cmd.action.name == "FORWARD" else ""
        print(f"{label}\n    -> {cmd.action.name:<10} speed={cmd.speed:.2f}{extra}"
              f"  [{cmd.reason}]\n")


if __name__ == "__main__":
    main()
