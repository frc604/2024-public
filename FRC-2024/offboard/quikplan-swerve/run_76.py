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


def plan(quiet):
    # Create the field
    field = Field(obstacles=False)

    # Create the robot model
    robot = Robot(
        MAX_RPM=4200,
        MAX_TORQUE=0.87,
    )

    # Configure the optimizer
    start_pose = (1.3, 2.4, np.pi)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
        use_simplified_constraints=False,
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    bottom_shot_waypoint = (2.8, 3.2, np.pi)
    qp.add_waypoint(
        bottom_shot_waypoint,
        10,
        end_constraints=[
            XYConstraint(bottom_shot_waypoint),
            GoalConstraint(),
            SpeedConstraint(0.5),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    avoid_bottom_stage_waypoint = (5.5, 1.7, np.pi)
    qp.add_waypoint(
        avoid_bottom_stage_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(avoid_bottom_stage_waypoint)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    avoid_bottom_stage_waypoint2 = (5.5, 1.7, np.pi)
    qp.add_waypoint(
        avoid_bottom_stage_waypoint2,
        10,
        end_constraints=[XYConstraint(avoid_bottom_stage_waypoint2)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    # approach 7
    waypoint7a = (7.9, 2.2, 1.15 * np.pi)
    qp.add_waypoint(
        waypoint7a,
        10,
        end_constraints=[PoseConstraint(waypoint7a)],
    )

    # get 7
    waypoint7 = (8.2, 2.4, 1.15 * np.pi)
    qp.add_waypoint(
        waypoint7,
        10,
        intermediate_constraints=[AngularConstraint(waypoint7)],
        end_constraints=[StoppedPoseConstraint(waypoint7)],
    )

    qp.add_waypoint(
        avoid_bottom_stage_waypoint2,
        10,
        end_constraints=[XYConstraint(avoid_bottom_stage_waypoint2)],
    )

    qp.add_waypoint(
        avoid_bottom_stage_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(avoid_bottom_stage_waypoint)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    qp.add_waypoint(
        bottom_shot_waypoint,
        10,
        end_constraints=[StoppedXYConstraint(bottom_shot_waypoint), GoalConstraint()],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    outside_stage_waypoint = (4.2, 3.2, np.pi)
    qp.add_waypoint(
        outside_stage_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(outside_stage_waypoint)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    inside_stage_waypoint = (5.0, 4.1, np.pi)
    qp.add_waypoint(
        inside_stage_waypoint,
        10,
        end_constraints=[PoseConstraint(inside_stage_waypoint)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    # get 6
    waypoint6 = (8.2, 4.1, np.pi)
    qp.add_waypoint(
        waypoint6,
        10,
        intermediate_constraints=[AngularConstraint(waypoint6), YConstraint(waypoint6)],
        end_constraints=[StoppedPoseConstraint(waypoint6)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    qp.add_waypoint(
        inside_stage_waypoint,
        10,
        intermediate_constraints=[YConstraint(inside_stage_waypoint)],
        end_constraints=[XYConstraint(inside_stage_waypoint)],
    )

    top_shot_waypoint = (4.3, 5.0, np.pi)
    qp.add_waypoint(
        top_shot_waypoint,
        10,
        end_constraints=[StoppedXYConstraint(top_shot_waypoint), GoalConstraint()],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    qp.add_waypoint(
        inside_stage_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(inside_stage_waypoint)],
    )

    qp.add_waypoint(
        waypoint6,
        10,
        end_constraints=[StoppedPoseConstraint(waypoint6)],
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "76")

    # Plot
    field.plot_traj(robot, traj, "76.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "76.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
