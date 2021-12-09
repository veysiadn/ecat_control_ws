## Header file directories : 
## /home/spinerobot/ros2_dashing/src/ros2/launch_ros/launch_ros/launch_ros/actions/lifecycle_node.py

from launch import LaunchDescription

## For lifecycle state transitions.
from launch_ros.actions import LifecycleNode
## For custom node launchs
from launch_ros.actions import Node

import ament_index_python
import launch
import launch_ros
import lifecycle_msgs
import os


def generate_launch_description():

    # Create pd node
    pd_node = launch_ros.actions.LifecycleNode(
        package = 'ecat_pkg',
        node_executable = 'ecat_node',
        node_name = 'ecat_node',
        output = 'screen',
        prefix = 'taskset -c 3'
    )

    # Make the pd node take the 'configure' transition
    pd_configure_event = launch.actions.EmitEvent(
            event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(pd_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    # Make the pd node take the 'activate' transition
    pd_activate_event = launch.actions.EmitEvent(
            event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(pd_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        )
    )

    # When the pd node reaches the 'inactive' state from the 'unconfigured' state, make it take the 'activate' transition
    pd_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = pd_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [pd_activate_event]
        )
    )

    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        pd_inactive_state_handler,
        pd_node,
        pd_configure_event,
        Node(package='controller', node_executable='joy_node', output='screen',prefix = 'taskset -c 2'),
        Node(package='custom_image_view', node_executable='custom_image_view', output='screen',prefix = 'taskset -c 2'),
        Node(package='usb_camera_driver', node_executable='usb_camera_driver_node', output='screen',prefix = 'taskset -c 2')
        ])