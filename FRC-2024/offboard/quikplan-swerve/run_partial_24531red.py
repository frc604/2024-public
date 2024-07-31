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


PICKUP4 = (8.2, 7.1, np.pi)
PICKUP5 = (8.2, 5.5, np.pi * 0.8)


def plan_2s4(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        # MAX_RPM=4500,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    start_pose = (1.3, 5.55, np.pi)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    # get piece 2
    waypoint2 = (2.6, 5.55, np.pi)
    qp.add_waypoint(
        waypoint2,
        10,
        end_constraints=[
            XYConstraint(waypoint2),
            GoalConstraint(),
            SpeedConstraint(0.5),
        ],
    )

    avoid_truss_waypoint_better = (5.4, 6.8, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint_better,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint_better)],
    )

    qp.add_waypoint(
        PICKUP4,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(PICKUP4)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/24531red-2s4")

    # Plot
    field.plot_traj(robot, traj, "partial/24531red-2s4.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/24531red-2s4.gif", save_gif=False)


def plan_4s5(quiet):
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
        PICKUP4,
        [StoppedXYConstraint(PICKUP4), GoalConstraint()],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    # drive into wing
    wing_shot_waypoint = (4.3, 6.3, np.pi)
    qp.add_waypoint(
        wing_shot_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(wing_shot_waypoint)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # approach 5
    waypoint5a = (5.0, 6.4, np.pi)
    qp.add_waypoint(
        waypoint5a,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint5a)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    waypoint5b = (7.0, 6.4, np.pi * 0.8)
    qp.add_waypoint(
        waypoint5b,
        10,
        end_constraints=[PoseConstraint(waypoint5b)],
    )

    # get 5
    qp.add_waypoint(
        PICKUP5,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP5)],
        end_constraints=[StoppedPoseConstraint(PICKUP5)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/24531red-4s5")

    # Plot
    field.plot_traj(robot, traj, "partial/24531red-4s5.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/24531red-4s5.gif", save_gif=False)


def plan_45(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        PICKUP4,
        [StoppedXYConstraint(PICKUP4), GoalConstraint()],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    # approach 5
    waypoint5a = (7.5, 6.5, np.pi * 0.8)
    qp.add_waypoint(
        waypoint5a,
        10,
        end_constraints=[PoseConstraint(waypoint5a)],
    )

    qp.add_waypoint(
        PICKUP5,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP5)],
        end_constraints=[StoppedPoseConstraint(PICKUP5)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/24531red-45")

    # Plot
    field.plot_traj(robot, traj, "partial/24531red-45.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/24531red-45.gif", save_gif=False)


def plan_5s3s1s(quiet):
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
        PICKUP5,
        [StoppedPoseConstraint(PICKUP5)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    # leave 5
    waypoint5b = (7.0, 6.4, np.pi * 0.8)
    qp.add_waypoint(
        waypoint5b,
        10,
        end_constraints=[XYConstraint(waypoint5b)],
    )

    betweenPose = (2.0, 5.55, np.pi)
    qp.add_waypoint(
        betweenPose,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(betweenPose), SpeedConstraint(0.3)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # get piece 3
    waypoint3 = (2.4, 4.3, np.pi)
    qp.add_waypoint(
        waypoint3,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(waypoint3)],
    )

    # approach note 1
    waypoint1a = (1.9, 5.8, np.pi)
    qp.add_waypoint(
        waypoint1a,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint1a)],
    )

    # get piece 1
    waypoint1 = (3.4, 6.5, np.pi)
    qp.add_waypoint(
        waypoint1,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(waypoint1)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/24531red-5s3s1s")

    # Plot
    field.plot_traj(robot, traj, "partial/24531red-5s3s1s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/24531red-5s3s1s.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan_2s4(args.quiet)
    plan_4s5(args.quiet)
    plan_45(args.quiet)
    plan_5s3s1s(args.quiet)
