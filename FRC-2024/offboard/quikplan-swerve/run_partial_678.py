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


PICKUP8 = (7.8, 0.8, np.pi)
PICKUP7 = (7.8, 2.8, 0.75 * np.pi)
PICKUP6 = (7.8, 4.1, np.pi)

OUTSIDE_STAGE_SHOT_WAYPOINT_A = (4.3, 4.9, np.pi)
OUTSIDE_STAGE_SHOT_WAYPOINT_B = (4.3, 5.2, np.pi)


def plan_6(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        # MAX_RPM=4500,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    start_pose = (1.3, 4.0, np.pi)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.RETRACT_AND_DISABLE_SHOOT),
    )

    bottom_shot_waypoint = (2.6, 3.1, np.pi)
    qp.add_waypoint(
        bottom_shot_waypoint,
        10,
        end_constraints=[
            XYConstraint(bottom_shot_waypoint),
            GoalConstraint(),
        ],
    )

    bottom_shot_waypoint2 = (2.8, 3.0, np.pi)
    qp.add_waypoint(
        bottom_shot_waypoint2,
        10,
        intermediate_constraints=[
            GoalConstraint(),
            SpeedConstraint(0.5),
        ],
        end_constraints=[
            XYConstraint(bottom_shot_waypoint2),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    oustside_stage = (4.0, 3.0, np.pi)
    qp.add_waypoint(
        oustside_stage,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(oustside_stage)],
    )

    inside_stage_deep = (4.9, 4.1, np.pi)
    qp.add_waypoint(
        inside_stage_deep,
        10,
        end_constraints=[PoseConstraint(inside_stage_deep)],
        end_action=Action(ActionType.DEPLOY_AND_DISABLE_SHOOT),
    )

    # get 6
    qp.add_waypoint(
        PICKUP6,
        10,
        intermediate_constraints=[
            AngularConstraint(PICKUP6),
            YConstraint(PICKUP6),
        ],
        end_constraints=[StoppedPoseConstraint(PICKUP6)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/678-6")

    # Plot
    field.plot_traj(robot, traj, "partial/678-6.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/678-6.gif", save_gif=False)


def plan_6s7(quiet):
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
        PICKUP6,
        [StoppedPoseConstraint(PICKUP6)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    middle_stage_waypoint = (5.5, 4.1, np.pi)
    qp.add_waypoint(
        middle_stage_waypoint,
        10,
        end_constraints=[XYConstraint(middle_stage_waypoint)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    qp.add_waypoint(
        OUTSIDE_STAGE_SHOT_WAYPOINT_A,
        10,
        end_constraints=[
            XYConstraint(OUTSIDE_STAGE_SHOT_WAYPOINT_A),
        ],
    )

    qp.add_waypoint(
        OUTSIDE_STAGE_SHOT_WAYPOINT_B,
        10,
        intermediate_constraints=[GoalConstraint(), SpeedConstraint(0.5)],
        end_constraints=[
            StoppedXYConstraint(OUTSIDE_STAGE_SHOT_WAYPOINT_B),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    quirky_inside_stage_waypoint = (5.3, 4.4, np.pi)
    qp.add_waypoint(
        quirky_inside_stage_waypoint,
        10,
        end_constraints=[XYConstraint(quirky_inside_stage_waypoint)],
        end_action=Action(ActionType.DEPLOY_AND_DISABLE_SHOOT),
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
    write_to_csv(traj, "partial/678-6s7")

    # Plot
    field.plot_traj(robot, traj, "partial/678-6s7.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/678-6s7.gif", save_gif=False)


def plan_67(quiet):
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
        PICKUP6,
        [StoppedPoseConstraint(PICKUP6)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    # approach 6
    waypoint6a = (7.3, 4.0, np.pi)
    qp.add_waypoint(
        waypoint6a,
        10,
        end_constraints=[PoseConstraint(waypoint6a)],
    )

    # get 6
    qp.add_waypoint(
        PICKUP7,
        10,
        end_constraints=[StoppedPoseConstraint(PICKUP7)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/678-67")

    # Plot
    field.plot_traj(robot, traj, "partial/678-67.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/678-67.gif", save_gif=False)


def plan_7s8(quiet):
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

    retract7waypoint = (5.5, 4.2, np.pi)
    qp.add_waypoint(
        retract7waypoint,
        10,
        end_constraints=[XYConstraint(retract7waypoint)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    qp.add_waypoint(
        OUTSIDE_STAGE_SHOT_WAYPOINT_A,
        10,
        end_constraints=[
            XYConstraint(OUTSIDE_STAGE_SHOT_WAYPOINT_A),
        ],
    )

    qp.add_waypoint(
        OUTSIDE_STAGE_SHOT_WAYPOINT_B,
        10,
        intermediate_constraints=[GoalConstraint(), SpeedConstraint(0.5)],
        end_constraints=[
            StoppedXYConstraint(OUTSIDE_STAGE_SHOT_WAYPOINT_B),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    inside_stage_waypoint = (4.6, 4.2, 0)
    qp.add_waypoint(
        inside_stage_waypoint,
        10,
        end_constraints=[XYConstraint(inside_stage_waypoint)],
    )

    avoid_truss_waypoint2 = (4.6, 2.45, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint2,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint2)],
    )

    avoid_truss_waypoint1 = (5.8, 1.5, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint1,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint1)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    # get 8
    qp.add_waypoint(
        PICKUP8,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP8)],
        end_constraints=[StoppedPoseConstraint(PICKUP8)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/678-7s8")

    # Plot
    field.plot_traj(robot, traj, "partial/678-7s8.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/678-7s8.gif", save_gif=False)


def plan_78(quiet):
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

    # approach 7
    waypoint7a = (7.3, 1.2, np.pi)
    qp.add_waypoint(
        waypoint7a,
        10,
        end_constraints=[PoseConstraint(waypoint7a)],
    )

    # get 7
    qp.add_waypoint(
        PICKUP8,
        10,
        end_constraints=[StoppedPoseConstraint(PICKUP8)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/678-78")

    # Plot
    field.plot_traj(robot, traj, "partial/678-78.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/678-78.gif", save_gif=False)


def plan_8s(quiet):
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
        PICKUP8,
        [StoppedPoseConstraint(PICKUP8)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    avoid_bottom_stage_waypoint = (6.2, 1.0, np.pi)
    qp.add_waypoint(
        avoid_bottom_stage_waypoint,
        10,
        end_constraints=[XYConstraint(avoid_bottom_stage_waypoint)],
    )

    bottom_shot_waypoint = (2.6, 3.1, np.pi)
    qp.add_waypoint(
        bottom_shot_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[
            StoppedXYConstraint(bottom_shot_waypoint),
        ],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/678-8s")

    # Plot
    field.plot_traj(robot, traj, "partial/678-8s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/678-8s.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    # plan_6(args.quiet)
    # plan_6s7(args.quiet)
    # plan_67(args.quiet)
    # plan_7s8(args.quiet)
    plan_78(args.quiet)
    # plan_8s(args.quiet)
