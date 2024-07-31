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
    XYConstraint,
    SpeedConstraint,
)
from utils.helpers import write_to_csv


def plan(quiet):
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
    waypoint1 = (2.6, 5.55, np.pi)
    qp.add_waypoint(
        waypoint1,
        10,
        end_constraints=[StoppedXYConstraint(waypoint1), GoalConstraint()],
    )

    # approach piece 3
    waypoint2 = (2.1, 4.8, np.pi)
    qp.add_waypoint(
        waypoint2,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint2)],
    )

    # get piece 3
    waypoint3 = (2.6, 4.3, np.pi)
    qp.add_waypoint(
        waypoint3,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(waypoint3)],
    )

    # approach note 3
    waypoint4 = (1.9, 5.8, np.pi)
    qp.add_waypoint(
        waypoint4,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint4)],
    )

    # get piece 1
    waypoint5 = (2.9, 6.5, np.pi)
    qp.add_waypoint(
        waypoint5,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(waypoint5)],
    )

    # # get 4
    # waypoint6 = (8.2, 7.4, np.pi)
    # qp.add_waypoint(
    #     waypoint6,
    #     10,
    #     intermediate_constraints=[GoalConstraint()],
    #     end_constraints=[StoppedXYConstraint(waypoint6)],
    #     end_action=Action(ActionType.DISABLE_SHOOT),
    # )

    # # drive into wing
    wing_shot_waypoint = (3.0, 6.3, np.pi)
    # qp.add_waypoint(
    #     wing_shot_waypoint,
    #     10,
    #     intermediate_constraints=[GoalConstraint()],
    #     end_constraints=[StoppedXYConstraint(wing_shot_waypoint)],
    #     end_action=Action(ActionType.ENABLE_SHOOT),
    # )

    # approach 5
    waypoint8a = (5.0, 6.4, np.pi)
    qp.add_waypoint(
        waypoint8a,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[XYConstraint(waypoint8a)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    waypoint8b = (7.0, 6.4, np.pi * 0.8)
    qp.add_waypoint(
        waypoint8b,
        10,
        end_constraints=[PoseConstraint(waypoint8b)],
    )

    # get 5
    waypoint8 = (8.2, 5.8, np.pi * 0.8)
    qp.add_waypoint(
        waypoint8,
        10,
        intermediate_constraints=[AngularConstraint(waypoint8)],
        end_constraints=[StoppedPoseConstraint(waypoint8)],
    )

    # leave 5
    qp.add_waypoint(
        waypoint8a,
        10,
        end_constraints=[PoseConstraint(waypoint8a)],
    )

    # drive into wing
    qp.add_waypoint(
        wing_shot_waypoint,
        10,
        end_constraints=[StoppedXYConstraint(wing_shot_waypoint), GoalConstraint()],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    # get 4
    waypoint6 = (8.2, 7.4, np.pi)
    qp.add_waypoint(
        waypoint6,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(waypoint6)],
        end_action=Action(ActionType.DISABLE_SHOOT),
    )

    # drive into wing
    wing_shot_waypoint = (3.5, 6.3, np.pi)
    qp.add_waypoint(
        wing_shot_waypoint,
        10,
        intermediate_constraints=[GoalConstraint()],
        end_constraints=[StoppedXYConstraint(wing_shot_waypoint)],
        end_action=Action(ActionType.ENABLE_SHOOT),
    )

    if not quiet:
        qp.plot_init()

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "23154")

    # Plot
    field.plot_traj(robot, traj, "23154.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "23154.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
