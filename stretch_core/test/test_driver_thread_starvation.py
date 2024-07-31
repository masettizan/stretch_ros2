import time
import pytest
import rclpy

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.stretch_driver import Client


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_thread_starvation():
    rclpy.init()
    node = Client("test_thread_starvation")

    # Create the goals
    goal_0 = {
        "joint_head_tilt": 0.0,
        "joint_head_pan": 0.0,
        "joint_wrist_roll": 0.0,
        "joint_wrist_pitch": 0.0,
        "joint_wrist_yaw": 0.0,
        "joint_lift": 1.1,
        "wrist_extension": 0.0,
    }
    goal_1 = {
        "joint_head_tilt": -0.02766916597432903,
        "joint_head_pan": 0.050683455439404626,
        "joint_wrist_roll": -0.006135923151542565,
        "joint_wrist_pitch": -0.15339807878856412,
        "joint_wrist_yaw": 0.0012783173232380344,
        "joint_lift": 0.5305833379477396,
        "wrist_extension": 0.45,
    }
    goal_2 = {
        "joint_head_tilt": -0.03766916597432903,
        "joint_head_pan": 0.060683455439404626,
        "joint_wrist_roll": -0.016135923151542565,
        "joint_wrist_pitch": -0.16339807878856412,
        "joint_wrist_yaw": 0.0112783173232380344,
        "joint_lift": 0.6305833379477396,
        "wrist_extension": 0.0,
    }

    # Switch to position mode
    node.get_logger().info("Switching to position mode...")
    node.change_mode("position")

    # Move to Goal 0
    node.get_logger().info("Moving to goal 0...")
    node.move_to_configuration(goal_0)

    # Move to Goal 1 asynchronously
    node.get_logger().info("Async moving to goal 1...")
    goal_1_future = node.move_to_configuration(goal_1, blocking=False)

    # Switch to navigation mode asynchronously
    node.get_logger().info("Async switching to navigation mode...")
    switch_to_nav_mode_future = node.change_mode("navigation", blocking=False)

    # Move to Goal 2 asynchronously
    time.sleep(0.25)
    node.get_logger().info("Async moving to goal 2...")
    goal_2_future = node.move_to_configuration(goal_2, blocking=False)

    # Wait for all futures to resolve
    node.get_logger().info("Waiting for async calls to finish...")
    while not all([f.done() for f in [goal_1_future, switch_to_nav_mode_future, goal_2_future]]):
        time.sleep(0.1)
    node.get_logger().info("Done!")

    # All calls should've resolved quickly (otherwise driver had thread starvation)
    assert False

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
