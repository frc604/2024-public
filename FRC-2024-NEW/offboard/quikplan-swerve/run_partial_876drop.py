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

START_POSE = (1.3, 1.6, np.pi)

APPROACH8 = (7.0, 1.15, 0.9 * np.pi)
PICKUP8 = (7.6, 0.95, 0.9 * np.pi)

APPROACH7 = (7.0, 2.0, 1.1 * np.pi)
PICKUP7 = (7.6, 2.5, 1.1 * np.pi)

APPROACH0 = (1.8, 1.6, 0.0)
PICKUP0 = (1.5, 1.6, 0.0)

APPROACH6 = (7.0, 3.7, 1.1 * np.pi)
PICKUP6 = (7.6, 4.1, 1.1 * np.pi)

BOTTOM_SHOT_WAYPOINT = (3.1, 3.1, np.pi)
BOTTOM_SHOT_WAYPOINTB = (3.2, 3.0, np.pi)

FINAL_BOTTOM_SHOT_WAYPOINT = (2.7, 3.1, np.pi)
FINAL_BOTTOM_SHOT_WAYPOINTB = (2.8, 3.0, np.pi)

AVOID_TRUSS_WAYPOINT = (5.8, 1.7, np.pi)

UNDER_STAGE_WAYPOINT = (5.5, 4.1, np.pi)
UNDER_STAGE_WAYPOINT2 = (4.0, 3.0, np.pi)


def plan_7(quiet):
    """
    Start position to pickup note 7
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        START_POSE,
        [StoppedPoseConstraint(START_POSE)],
        start_action=Action(ActionType.TURN_OFF_LAUNCHER),
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    # approach 7
    qp.add_waypoint(
        APPROACH7,
        10,
        end_constraints=[PoseConstraint(APPROACH7)],
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
    write_to_csv(traj, "partial/876drop-7")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-7.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-7.gif", save_gif=False)


def plan_8(quiet):
    """
    Start position to pick up note 8
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        START_POSE,
        [StoppedPoseConstraint(START_POSE)],
        start_action=Action(ActionType.TURN_OFF_LAUNCHER),
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    qp.add_waypoint(
        APPROACH8,
        10,
        end_constraints=[PoseConstraint(APPROACH8)],
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
    write_to_csv(traj, "partial/876drop-8")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-8.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-8.gif", save_gif=False)


def plan_86(quiet):
    """
    miss 8 to pick up 6
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        APPROACH6,
        10,
        end_constraints=[PoseConstraint(APPROACH6)],
    )

    qp.add_waypoint(
        PICKUP6,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP6)],
        end_constraints=[
            StoppedPoseConstraint(PICKUP6),
        ],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-86")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-86.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-86.gif", save_gif=False)


def plan_87(quiet):
    """
    miss 8 to pick up 7
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        APPROACH7,
        10,
        end_constraints=[PoseConstraint(APPROACH7)],
    )

    qp.add_waypoint(
        PICKUP7,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP7)],
        end_constraints=[
            StoppedPoseConstraint(PICKUP7),
        ],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-87")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-87.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-87.gif", save_gif=False)


def plan_78(quiet):
    """
    miss 7 to pick up 8
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    # approach 8
    qp.add_waypoint(
        APPROACH8,
        10,
        end_constraints=[PoseConstraint(APPROACH8)],
    )

    # pickup 8
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
    write_to_csv(traj, "partial/876drop-78")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-78.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-78.gif", save_gif=False)


def plan_76(quiet):
    """
    miss 7 to pick up 6
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    # approach 6
    qp.add_waypoint(
        APPROACH6,
        10,
        end_constraints=[PoseConstraint(APPROACH6)],
    )

    # pickup 6
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
    write_to_csv(traj, "partial/876drop-76")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-76.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-76.gif", save_gif=False)


def plan_67(quiet):
    """
    miss 6 to pick up 7
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        APPROACH7,
        10,
        end_constraints=[PoseConstraint(APPROACH7)],
    )

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
    write_to_csv(traj, "partial/876drop-67")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-67.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-67.gif", save_gif=False)


def plan_68(quiet):
    """
    miss 6 to pick up 8
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        APPROACH8,
        10,
        end_constraints=[PoseConstraint(APPROACH8)],
    )

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
    write_to_csv(traj, "partial/876drop-68")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-68.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-67.gif", save_gif=False)


def plan_80(quiet):
    """
    miss 8 to pick up 0
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        APPROACH0,
        10,
        end_constraints=[PoseConstraint(APPROACH0)],
    )

    qp.add_waypoint(
        PICKUP0,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP0)],
        end_constraints=[
            StoppedPoseConstraint(PICKUP0),
        ],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-80")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-80.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-80.gif", save_gif=False)


def plan_70(quiet):
    """
    miss 7 to pick up 0
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        AVOID_TRUSS_WAYPOINT,
        10,
        end_constraints=[
            XYConstraint(AVOID_TRUSS_WAYPOINT),
        ],
    )

    qp.add_waypoint(
        APPROACH0,
        10,
        end_constraints=[
            PoseConstraint(APPROACH0),
        ],
    )

    qp.add_waypoint(
        PICKUP0,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP0)],
        end_constraints=[
            StoppedPoseConstraint(PICKUP0),
        ],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-70")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-70.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-70.gif", save_gif=False)


def plan_60(quiet):
    """
    miss 6 to pick up 0
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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
        start_action=Action(ActionType.RETRACT_AND_DISABLE_SHOOT),
    )

    qp.add_waypoint(
        UNDER_STAGE_WAYPOINT,
        10,
        end_constraints=[
            XYConstraint(UNDER_STAGE_WAYPOINT),
            AngularConstraint(UNDER_STAGE_WAYPOINT),
        ],
    )

    qp.add_waypoint(
        UNDER_STAGE_WAYPOINT2,
        10,
        end_constraints=[XYConstraint(UNDER_STAGE_WAYPOINT2)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    qp.add_waypoint(
        APPROACH0,
        10,
        end_constraints=[PoseConstraint(APPROACH0)],
    )

    qp.add_waypoint(
        PICKUP0,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP0)],
        end_constraints=[StoppedPoseConstraint(PICKUP0)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-60")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-60.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-60.gif", save_gif=False)


def plan_8s(quiet):
    """
    pick up 8 to score
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        AVOID_TRUSS_WAYPOINT,
        10,
        end_constraints=[XYConstraint(AVOID_TRUSS_WAYPOINT)],
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
            StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-8s")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-8s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-8s.gif", save_gif=False)


def plan_7s(quiet):
    """
    pick up 7 to score
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        AVOID_TRUSS_WAYPOINT,
        10,
        end_constraints=[XYConstraint(AVOID_TRUSS_WAYPOINT)],
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
            StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-7s")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-7s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-7s.gif", save_gif=False)


def plan_6s(quiet):
    """
    pick up 6 to score
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
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

    qp.add_waypoint(
        UNDER_STAGE_WAYPOINT,
        10,
        end_constraints=[XYConstraint(UNDER_STAGE_WAYPOINT)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    qp.add_waypoint(
        UNDER_STAGE_WAYPOINT2,
        10,
        end_constraints=[XYConstraint(UNDER_STAGE_WAYPOINT2)],
    )

    qp.add_waypoint(
        BOTTOM_SHOT_WAYPOINT,
        10,
        end_constraints=[
            StoppedXYConstraint(BOTTOM_SHOT_WAYPOINT),
        ],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    qp.add_waypoint(
        BOTTOM_SHOT_WAYPOINTB,
        10,
        intermediate_constraints=[SpeedConstraint(0.5), GoalConstraint()],
        end_constraints=[
            StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-6s")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-6s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-6s.gif", save_gif=False)


def plan_0s(quiet):
    """
    Pick up 0 to score
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=4500,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        PICKUP0,
        [StoppedPoseConstraint(PICKUP0)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    qp.add_waypoint(
        FINAL_BOTTOM_SHOT_WAYPOINT,
        10,
        end_constraints=[
            StoppedXYConstraint(FINAL_BOTTOM_SHOT_WAYPOINT),
        ],
    )

    qp.add_waypoint(
        FINAL_BOTTOM_SHOT_WAYPOINTB,
        10,
        intermediate_constraints=[SpeedConstraint(0.5), GoalConstraint()],
        end_constraints=[
            StoppedXYConstraint(FINAL_BOTTOM_SHOT_WAYPOINTB),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # Drive towards centerline
    qp.add_waypoint(
        AVOID_TRUSS_WAYPOINT,
        10,
        end_constraints=[XYConstraint(AVOID_TRUSS_WAYPOINT)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    end_pos = (7.1, 1.5, np.pi)
    qp.add_waypoint(
        end_pos,
        10,
        end_constraints=[StoppedPoseConstraint(end_pos)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-0s")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-0s.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-0s.gif", save_gif=False)


def plan_s8(quiet):
    """
    score to pick up 8
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        BOTTOM_SHOT_WAYPOINTB,
        [StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB), GoalConstraint()],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    qp.add_waypoint(
        AVOID_TRUSS_WAYPOINT,
        10,
        end_constraints=[
            XYConstraint(AVOID_TRUSS_WAYPOINT),
        ],
    )

    qp.add_waypoint(
        APPROACH8,
        10,
        end_constraints=[
            PoseConstraint(APPROACH8),
        ],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    qp.add_waypoint(
        PICKUP8,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP8)],
        end_constraints=[
            StoppedPoseConstraint(PICKUP8),
        ],
    )
    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-s8")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-s8.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-s8.gif", save_gif=False)


def plan_s7(quiet):
    """
    score to pick up 7
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        BOTTOM_SHOT_WAYPOINTB,
        [StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB), GoalConstraint()],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    qp.add_waypoint(
        AVOID_TRUSS_WAYPOINT,
        10,
        end_constraints=[
            XYConstraint(AVOID_TRUSS_WAYPOINT),
        ],
    )

    qp.add_waypoint(
        APPROACH7,
        10,
        end_constraints=[
            PoseConstraint(APPROACH7),
        ],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    qp.add_waypoint(
        PICKUP7,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP7)],
        end_constraints=[
            StoppedPoseConstraint(PICKUP7),
        ],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-s7")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-s7.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-s7.gif", save_gif=False)


def plan_s6(quiet):
    """
    score to pick up 6
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        BOTTOM_SHOT_WAYPOINTB,
        [StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB), GoalConstraint()],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.RETRACT_INTAKE),
    )

    qp.add_waypoint(
        UNDER_STAGE_WAYPOINT2,
        10,
        end_constraints=[XYConstraint(UNDER_STAGE_WAYPOINT2)],
    )

    qp.add_waypoint(
        UNDER_STAGE_WAYPOINT,
        10,
        end_constraints=[XYConstraint(UNDER_STAGE_WAYPOINT)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    qp.add_waypoint(
        APPROACH6,
        10,
        end_constraints=[
            PoseConstraint(APPROACH6),
        ],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )
    qp.add_waypoint(
        PICKUP6,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP6)],
        end_constraints=[
            StoppedPoseConstraint(PICKUP6),
        ],
    )
    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-s6")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-s6.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-s6.gif", save_gif=False)


def plan_s0(quiet):
    """
    score to pick up 0
    """
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=3800,
        # MAX_TORQUE=0.8,
    )

    # Configure the optimizer
    qp = QuikPlan(
        field,
        robot,
        BOTTOM_SHOT_WAYPOINTB,
        [StoppedXYConstraint(BOTTOM_SHOT_WAYPOINTB), GoalConstraint()],
        apply_boundaries=False,
        use_simplified_constraints=False,
    )

    qp.add_waypoint(
        APPROACH0,
        10,
        end_constraints=[PoseConstraint(APPROACH0)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )
    qp.add_waypoint(
        PICKUP0,
        10,
        intermediate_constraints=[AngularConstraint(PICKUP0)],
        end_constraints=[StoppedPoseConstraint(PICKUP0)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "partial/876drop-s0")

    # Plot
    field.plot_traj(robot, traj, "partial/876drop-s0.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "partial/876drop-s0.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])

    # Start pose to each note
    plan_7(args.quiet)
    plan_8(args.quiet)

    # Recovery trajectories
    plan_60(args.quiet)
    plan_67(args.quiet)
    plan_68(args.quiet)
    plan_70(args.quiet)
    plan_76(args.quiet)
    plan_78(args.quiet)
    plan_80(args.quiet)
    plan_86(args.quiet)
    plan_87(args.quiet)

    # Pickup to score
    plan_0s(args.quiet)
    plan_6s(args.quiet)
    plan_7s(args.quiet)
    plan_8s(args.quiet)

    # Score to pickup
    plan_s0(args.quiet)
    plan_s6(args.quiet)
    plan_s7(args.quiet)
    plan_s8(args.quiet)
