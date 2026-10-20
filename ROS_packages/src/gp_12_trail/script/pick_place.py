#!/usr/bin/env python3

import argparse
import sys

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder


ARM_GROUP = 'gp12_arm'
GRIPPER_GROUP = 'gp12_gripper'
TOOL_LINK = 'tool0'
BASE_FRAME = 'base_link'

PICK_POSITION = (0.90, 0.50, 0.75)
PLACE_POSITION = (0.90, -0.50, 0.75)
APPROACH_HEIGHT = 0.15
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.7


def make_pose(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    return pose


def plan_and_execute(moveit, planning_component, group_name, label):
    planning_component.set_start_state_to_current_state()
    plan_parameters = PlanRequestParameters(moveit)
    plan_parameters.planning_pipeline = 'ompl'
    plan_parameters.planning_time = 5.0
    plan = planning_component.plan(plan_parameters)
    if not plan:
        raise RuntimeError(f'Planning failed for {label}')

    print(f'Executing: {label}')
    moveit.execute(group_name, plan.trajectory, blocking=True)


def move_arm(moveit, arm, position, label):
    arm.set_goal_state(
        pose_stamped_msg=make_pose(*position),
        pose_link=TOOL_LINK,
    )
    plan_and_execute(moveit, arm, ARM_GROUP, label)


def move_gripper(moveit, gripper, position, label):
    target_state = gripper.get_start_state()
    target_state.set_joint_group_positions(GRIPPER_GROUP, [position])
    gripper.set_goal_state(robot_state=target_state)
    plan_and_execute(moveit, gripper, GRIPPER_GROUP, label)


def run_pick_and_place():
    config = MoveItConfigsBuilder(
        'motoman_gp12', package_name='gp12_moveit_config'
    ).to_moveit_configs()
    moveit_parameters = config.to_dict()
    moveit_parameters['planning_pipelines'] = {
        'pipeline_names': moveit_parameters['planning_pipelines'],
    }
    moveit = MoveItPy(node_name='gp12_pick_place', config_dict=moveit_parameters)

    arm = moveit.get_planning_component(ARM_GROUP)
    gripper = moveit.get_planning_component(GRIPPER_GROUP)

    pick_approach = (PICK_POSITION[0], PICK_POSITION[1], PICK_POSITION[2] + APPROACH_HEIGHT)
    place_approach = (PLACE_POSITION[0], PLACE_POSITION[1], PLACE_POSITION[2] + APPROACH_HEIGHT)

    move_gripper(moveit, gripper, GRIPPER_OPEN, 'open gripper')
    move_arm(moveit, arm, pick_approach, 'approach pick position')
    move_arm(moveit, arm, PICK_POSITION, 'lower to pick position')
    move_gripper(moveit, gripper, GRIPPER_CLOSED, 'close gripper')
    move_arm(moveit, arm, pick_approach, 'lift object')
    move_arm(moveit, arm, place_approach, 'move above place position')
    move_arm(moveit, arm, PLACE_POSITION, 'lower to place position')
    move_gripper(moveit, gripper, GRIPPER_OPEN, 'release object')
    move_arm(moveit, arm, place_approach, 'retreat from place position')
    moveit.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Plan and execute a sample GP12 pick-and-place sequence.'
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        help='Execute robot motion. Without this flag, no motion is commanded.',
    )
    args = parser.parse_args()

    if not args.execute:
        print('No motion commanded. Review the sample coordinates, then rerun with --execute.')
        return 0

    try:
        run_pick_and_place()
    except Exception as error:
        print(f'Pick-and-place failed: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())