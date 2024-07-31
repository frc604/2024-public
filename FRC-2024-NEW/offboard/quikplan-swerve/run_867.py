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
        MAX_RPM=4500,
        MAX_TORQUE=0.8,
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
        start_action=Action(ActionType.DISABLE_SHOOT),
    )

    bottom_shot_waypoint = (2.3, 3.4, np.pi)
    qp.add_waypoint(
        bottom_shot_waypoint,
        10,
        end_constraints=[
            StoppedXYConstraint(bottom_shot_waypoint),
            GoalConstraint(),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    avoid_bottom_stage_waypoint = (6.2, 1.0, np.pi)
    qp.add_waypoint(
        avoid_bottom_stage_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(avoid_bottom_stage_waypoint)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    # approach 8
    waypoint8a = (7.1, 0.8, np.pi)
    qp.add_waypoint(
        waypoint8a,
        10,
        end_constraints=[XYConstraint(waypoint8a)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    # get 8
    waypoint8 = (7.8, 0.8, np.pi)
    qp.add_waypoint(
        waypoint8,
        10,
        intermediate_constraints=[AngularConstraint(waypoint8), YConstraint(waypoint8)],
        end_constraints=[StoppedPoseConstraint(waypoint8)],
    )

    avoid_truss_waypoint1 = (5.8, 1.5, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint1,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint1)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    avoid_truss_waypoint2 = (4.6, 2.45, np.pi)
    qp.add_waypoint(
        avoid_truss_waypoint2,
        10,
        end_constraints=[XYConstraint(avoid_truss_waypoint2)],
    )

    inside_stage_waypoint = (4.6, 4.2, 0)
    qp.add_waypoint(
        inside_stage_waypoint,
        10,
        end_constraints=[XYConstraint(inside_stage_waypoint)],
    )

    outside_stage_shot_waypoint = (4.3, 4.9, np.pi)
    qp.add_waypoint(
        outside_stage_shot_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[
            StoppedXYConstraint(outside_stage_shot_waypoint),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    inside_stage_deep = (4.9, 4.1, np.pi)
    qp.add_waypoint(
        inside_stage_deep,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(inside_stage_deep)],
        end_action=Action(ActionType.DEPLOY_AND_DISABLE_SHOOT),
    )

    # get 6
    waypoint6 = (7.8, 4.1, np.pi)
    qp.add_waypoint(
        waypoint6,
        10,
        intermediate_constraints=[
            YConstraint(waypoint6),
        ],
        end_constraints=[StoppedPoseConstraint(waypoint6)],
    )

    middle_stage_waypoint = (5.5, 4.1, np.pi)
    qp.add_waypoint(
        middle_stage_waypoint,
        10,
        end_constraints=[XYConstraint(middle_stage_waypoint)],
        end_action=Action(ActionType.RETRACT_AND_DISABLE_SHOOT),
    )

    qp.add_waypoint(
        outside_stage_shot_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[
            StoppedXYConstraint(outside_stage_shot_waypoint),
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
    waypoint7 = (7.8, 2.4, 0.75 * np.pi)
    qp.add_waypoint(
        waypoint7,
        10,
        intermediate_constraints=[AngularConstraint(waypoint7)],
        end_constraints=[StoppedPoseConstraint(waypoint7)],
    )
    retract7waypoint = (5.5, 4.2, np.pi)
    qp.add_waypoint(
        retract7waypoint,
        10,
        end_constraints=[XYConstraint(retract7waypoint)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )
    qp.add_waypoint(
        outside_stage_shot_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[
            StoppedXYConstraint(outside_stage_shot_waypoint),
        ],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "867")

    # Plot
    field.plot_traj(robot, traj, "867.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "867.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
