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


PICKUP1 = (2.9, 6.7, np.pi)
PICKUP4 = (8.2, 7.4, np.pi)
PICKUP5 = (8.2, 5.8, np.pi * 0.8)
PICKUP6 = (8.2, 4.2, np.pi * 0.7)
WING_SHOT_WAYPOINT = (4.3, 6.3, np.pi)


def plan_1s4(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        # MAX_RPM=4500,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    start_pose = (0.7, 6.8, np.deg2rad(240))
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    # get piece 1
    qp.add_waypoint(
        PICKUP1,
        10,
        end_constraints=[
            XYConstraint(PICKUP1),
            SpeedConstraint(0.5),
        ],
    )

    avoid_truss_waypoint_better = (5.4, 6.8, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint_better,
        10,
        intermediate_constraints=[GoalConstraint()],
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
    write_to_csv(traj, "partial/1456-1s4")

    # Plot
    field.plot_traj(robot, traj, "partial/1456-1s4.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/1456-1s4.gif", save_gif=False)


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
    qp.add_waypoint(
        WING_SHOT_WAYPOINT,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(WING_SHOT_WAYPOINT)],
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
    write_to_csv(traj, "partial/1456-4s5")

    # Plot
    field.plot_traj(robot, traj, "partial/1456-4s5.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/1456-4s5.gif", save_gif=False)


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
    write_to_csv(traj, "partial/1456-45")

    # Plot
    field.plot_traj(robot, traj, "partial/1456-45.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/1456-45.gif", save_gif=False)


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
    write_to_csv(traj, "partial/1456-5s6")

    # Plot
    field.plot_traj(robot, traj, "partial/1456-5s6.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/1456-5s6.gif", save_gif=False)


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
    write_to_csv(traj, "partial/1456-56")

    # Plot
    field.plot_traj(robot, traj, "partial/1456-56.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/1456-56.gif", save_gif=False)


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
    write_to_csv(traj, "partial/1456-6s")

    # Plot
    field.plot_traj(robot, traj, "partial/1456-6s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/1456-6s.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan_1s4(args.quiet)
    plan_4s5(args.quiet)
    plan_45(args.quiet)
    plan_5s6(args.quiet)
    plan_56(args.quiet)
    plan_6s(args.quiet)
