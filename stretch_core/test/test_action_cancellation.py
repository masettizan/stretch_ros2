import time
import pytest
import rclpy

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.stretch_driver import Client


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_position_mode_cancellation():
    rclpy.init()
    node = Client("test_fjt_cancel")

    # Move the arm lift all the way up
    print(node.mode)
    node.move_to_configuration({"joint_lift": 1.1})

    # Move the lift part way down
    node.move_to_configuration({"joint_lift": 0.5}, blocking=False)

    # Sleep for 1s, then cancel
    node.get_logger().info("Sleeping for 1s")
    time.sleep(1)
    node.get_logger().info("Cancelling the goal")
    node.cancel_goal()

    # Lift position should not be near 0.5
    assert node.q_curr["joint_lift"] > 0.8

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
