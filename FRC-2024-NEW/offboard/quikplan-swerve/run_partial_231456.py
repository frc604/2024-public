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


PICKUP1 = (2.9, 6.5, np.pi)
PICKUP4 = (8.2, 7.4, np.pi)
PICKUP5 = (8.2, 5.8, np.pi * 0.8)
PICKUP6 = (8.2, 4.2, np.pi * 0.7)
WING_SHOT_WAYPOINT = (3.0, 6.3, np.pi)


def plan_2s3s1s(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

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
        end_constraints=[StoppedXYConstraint(waypoint2), GoalConstraint()],
    )

    # approach piece 3
    waypoint3a = (2.1, 4.8, np.pi)
    qp.add_waypoint(
        waypoint3a,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint3a)],
    )

    # get piece 3
    waypoint3 = (2.6, 4.3, np.pi)
    qp.add_waypoint(
        waypoint3,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(waypoint3)],
    )

    # approach note 1
    waypoint1a = (1.9, 6.1, np.pi)
    qp.add_waypoint(
        waypoint1a,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint1a)],
    )

    # get piece 1
    qp.add_waypoint(
        PICKUP1,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(PICKUP1)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/231456-2s3s1s")

    # Plot
    field.plot_traj(robot, traj, "partial/231456-2s3s1s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/231456-2s3s1s.gif", save_gif=False)


def plan_4s5(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        PICKUP4,
        [StoppedPoseConstraint(PICKUP4)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    # drive into wing
    qp.add_waypoint(
        WING_SHOT_WAYPOINT,
        10,
        end_constraints=[StoppedXYConstraint(WING_SHOT_WAYPOINT), GoalConstraint()],
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
    write_to_csv(traj, "partial/231456-4s5")

    # Plot
    field.plot_traj(robot, traj, "partial/231456-4s5.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/231456-4s5.gif", save_gif=False)


def plan_5s6(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

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

    # avoid truss
    waypoint = (6.1, 6.3, np.pi)
    qp.add_waypoint(
        waypoint,
        10,
        end_constraints=[XYConstraint(waypoint)],
    )

    # drive into wing
    qp.add_waypoint(
        WING_SHOT_WAYPOINT,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(WING_SHOT_WAYPOINT)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # avoid truss
    waypoint = (6.1, 6.3, np.pi)
    qp.add_waypoint(
        waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint)],
    )

    # approach 6
    waypoint6a = (7.0, 5.5, np.pi * 0.7)
    qp.add_waypoint(
        waypoint6a,
        10,
        end_constraints=[PoseConstraint(waypoint6a)],
    )

    qp.add_waypoint(
        PICKUP6,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP6)],
        end_constraints=[StoppedPoseConstraint(PICKUP6)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/231456-5s6")

    # Plot
    field.plot_traj(robot, traj, "partial/231456-5s6.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/231456-5s6.gif", save_gif=False)


def plan_6s(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        PICKUP6,
        [StoppedPoseConstraint(PICKUP6)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    # avoid truss
    waypoint = (6.1, 6.3, np.pi)
    qp.add_waypoint(
        waypoint,
        10,
        end_constraints=[XYConstraint(waypoint)],
    )

    # drive into wing
    qp.add_waypoint(
        WING_SHOT_WAYPOINT,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(WING_SHOT_WAYPOINT)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/231456-6s")

    # Plot
    field.plot_traj(robot, traj, "partial/231456-6s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/231456-6s.gif", save_gif=False)


def plan_14(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        PICKUP1,
        [StoppedXYConstraint(PICKUP1)],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    # approach 4
    waypoint4a = (6.2, 7.4, np.pi)
    qp.add_waypoint(
        waypoint4a,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint4a)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    # get 4
    qp.add_waypoint(
        PICKUP4,
        10,
        end_constraints=[StoppedPoseConstraint(PICKUP4)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/231456-14")

    # Plot
    field.plot_traj(robot, traj, "partial/231456-14.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/231456-14.gif", save_gif=False)


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
        [StoppedPoseConstraint(PICKUP4)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    # approach 5
    waypoint5a = (7.8, 7.0, np.pi * 0.8)
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
    write_to_csv(traj, "partial/231456-45")

    # Plot
    field.plot_traj(robot, traj, "partial/231456-45.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/231456-45.gif", save_gif=False)


def plan_56(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot()

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

    # approach 6
    waypoint6a = (7.8, 5.5, np.pi * 0.7)
    qp.add_waypoint(
        waypoint6a,
        10,
        end_constraints=[PoseConstraint(waypoint6a)],
    )

    qp.add_waypoint(
        PICKUP6,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP6)],
        end_constraints=[StoppedPoseConstraint(PICKUP6)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/231456-56")

    # Plot
    field.plot_traj(robot, traj, "partial/231456-56.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/231456-56.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan_2s3s1s(args.quiet)
    plan_4s5(args.quiet)
    plan_5s6(args.quiet)
    plan_6s(args.quiet)
    plan_14(args.quiet)
    plan_45(args.quiet)
    plan_56(args.quiet)
