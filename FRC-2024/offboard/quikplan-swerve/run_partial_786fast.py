import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np

from utils.field import Field
from utils.robot import Robot
from utils.quikplan import (
    Action,
    ActionType,
    AngularConstraint,
    GoalConstraint,
    PoseConstraint,
    QuikPlan,
    StoppedPoseConstraint,
    StoppedXYConstraint,
    YConstraint,
    XYConstraint,
    SpeedConstraint,
    AngularConstraint,
)
from utils.helpers import write_to_csv


PICKUP8 = (7.8, 0.8, 0.9 * np.pi)
PICKUP7 = (7.8, 2.0, 1.2 * np.pi)
PICKUP6 = (7.8, 4.1, np.pi)

BOTTOM_SHOT_WAYPOINT = (2.6, 3.1, np.pi)
BOTTOM_SHOT_WAYPOINTB = (2.8, 3.0, np.pi)

OUTSIDE_STAGE_SHOT_WAYPOINT_A = (4.3, 4.9, np.pi)
OUTSIDE_STAGE_SHOT_WAYPOINT_B = (4.3, 5.2, np.pi)


def plan_7(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    start_pose = (0.8, 4.1, np.deg2rad(120))
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    bottom_shot_waypoint = (2.6, 3.1, np.pi)
    qp.add_waypoint(
        bottom_shot_waypoint,
        10,
        end_constraints=[
            XYConstraint(bottom_shot_waypoint),
            GoalConstraint(),
        ],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    avoid_truss_waypoint1 = (5.8, 1.2, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint1,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint1)],
    )

    # approach 7
    waypoint7a = (7.1, 1.6, 1.2 * np.pi)
    qp.add_waypoint(
        waypoint7a,
        10,
        end_constraints=[XYConstraint(waypoint7a)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    # get 7
    qp.add_waypoint(
        PICKUP7,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP7)],
        end_constraints=[StoppedPoseConstraint(PICKUP7)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/786fast-7")

    # Plot
    field.plot_traj(robot, traj, "partial/786fast-7.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/786fast-7.gif", save_gif=False)


def plan_7s8s(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        # MAX_RPM=4500,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        PICKUP7,
        [StoppedPoseConstraint(PICKUP7)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    avoid_truss_waypoint1 = (5.8, 1.4, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint1,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint1)],
    )

    qp.add_waypoint(
        BOTTOM_SHOT_WAYPOINT,
        10,
        end_constraints=[
            StoppedXYConstraint(BOTTOM_SHOT_WAYPOINT),
        ],
    )

    qp.add_waypoint(
        BOTTOM_SHOT_WAYPOINTB,
        10,
        intermediate_constraints=[SpeedConstraint(0.5), GoalConstraint()],
        end_constraints=[
            XYConstraint(BOTTOM_SHOT_WAYPOINTB),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # approach 8
    waypoint8a = (7.1, 1.0, 0.9 * np.pi)
    qp.add_waypoint(
        waypoint8a,
        10,
        end_constraints=[XYConstraint(waypoint8a)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    # get 8
    qp.add_waypoint(
        PICKUP8,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP8)],
        end_constraints=[StoppedPoseConstraint(PICKUP8)],
    )

    qp.add_waypoint(
        avoid_truss_waypoint1,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint1)],
    )

    qp.add_waypoint(
        BOTTOM_SHOT_WAYPOINT,
        10,
        end_constraints=[
            StoppedXYConstraint(BOTTOM_SHOT_WAYPOINT),
        ],
    )

    qp.add_waypoint(
        BOTTOM_SHOT_WAYPOINTB,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[
            StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/786fast-7s8s")

    # Plot
    field.plot_traj(robot, traj, "partial/786fast-7s8s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/786fast-7s8s.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan_7(args.quiet)
    plan_7s8s(args.quiet)
    # plan_87(args.quiet)
    # plan_7s6(args.quiet)
    # plan_76(args.quiet)
    # plan_6s(args.quiet)
