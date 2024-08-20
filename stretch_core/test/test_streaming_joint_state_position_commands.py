import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from hello_helpers.hello_misc import LoopTimer
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from hello_helpers.gripper_conversion import GripperConversion
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import time



class JointPosePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('float_array_publisher')
        self.reentrant_cb = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(JointState, 'joint_pose_cmd', 10, callback_group=self.reentrant_cb)
        # subscribe to joint states
        self.joint_state = JointState()
        self.joint_states_subscriber = self.create_subscription(JointState, 
                                                                '/stretch/joint_states', 
                                                                self.joint_states_callback, 10,callback_group=self.reentrant_cb)
        self.switch_to_position_mode_service = self.create_client(Trigger, '/switch_to_position_mode',callback_group=self.reentrant_cb)

        self.switch_to_navigation_mode_service = self.create_client(Trigger, '/switch_to_navigation_mode',callback_group=self.reentrant_cb)

        self.activate_streaming_position_service = self.create_client(Trigger, '/activate_streaming_position', callback_group=self.reentrant_cb)
        self.deactivate_streaming_position_service = self.create_client(Trigger, '/deactivate_streaming_position', callback_group=self.reentrant_cb)

        while not self.switch_to_position_mode_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/switch_to_position_mode' service...")
        self.gripper_conversion = GripperConversion()
        self.lt = LoopTimer(print_debug=True)

    def joint_states_callback(self, msg):
        self.joint_state = msg

    def switch_to_position_mode(self):
        trigger_request = Trigger.Request()
        trigger_result = self.switch_to_position_mode_service.call_async(trigger_request)
        return

    def activate_streaming_position(self):
        trigger_request = Trigger.Request()
        trigger_result = self.activate_streaming_position_service.call_async(trigger_request)
        time.sleep(1)

    def deactivate_streaming_position(self):
        trigger_request = Trigger.Request()
        trigger_result = self.deactivate_streaming_position_service.call_async(trigger_request)
        return


    def switch_to_navigation_mode(self):
        trigger_request = Trigger.Request()
        trigger_result = self.switch_to_navigation_mode_service.call_async(trigger_request)
        return

    def parse_joint_state(self, joint_state_msg):
        joint_status = {}
        for name, position in zip(joint_state_msg.name, joint_state_msg.position):
            joint_status[name] = position
        return joint_status

    def publish_joint_pose(self, joint_pose):
        self.publisher_.publish(joint_pose)
        self.get_logger().info('Publishing: "%s"' % joint_pose)

if __name__ == '__main__':
    joint_pose_publisher = JointPosePublisher()
    rclpy.spin_once(joint_pose_publisher)

    # joint_pose_publisher.switch_to_navigation_mode()
    joint_pose_publisher.activate_streaming_position()

    home_pose = {
        'joint_lift': 0.6,
        'joint_arm': 0.0,
        'joint_wrist_yaw': 0.0,
        'joint_wrist_pitch': 0.0,
        'joint_wrist_roll': 0.0,
        'joint_head_pan': 0.0,
        'joint_head_tilt': 0.0,
        # 'joint_base_translate',
        # 'joint_base_rotate',
        'joint_gripper': 0
    }
    goal = JointState()
    for name, pos in home_pose.items():
        goal.name.append(name)
        goal.position.append(pos)
    joint_pose_publisher.publish_joint_pose(goal)
    time.sleep(3)

    start = time.time()
    target_rate = 30

    i = 0
    joint_pose_publisher.lt.reset()
    while i<100:
        start = time.time()
        i = 1 + i
        rclpy.spin_once(joint_pose_publisher)
        j_state = joint_pose_publisher.joint_state
        goal = JointState()
        for name, pos in zip(j_state.name, j_state.position):
            if name == 'joint_lift':
                goal.name.append('joint_lift')
                goal.position.append(pos + 0.05)
            elif name == "wrist_extension":
                goal.name.append('joint_arm')
                goal.position.append(pos + 0.05)
            elif name == 'joint_wrist_pitch':
                goal.name.append('joint_wrist_pitch')
                goal.position.append(pos + 0.1)
            elif name == 'joint_wrist_roll':
                goal.name.append('joint_wrist_roll')
                goal.position.append(pos + 0.1)
            elif name == 'joint_wrist_yaw':
                goal.name.append('joint_wrist_yaw')
                goal.position.append(pos + 0.1)
            elif name == 'joint_head_pan':
                goal.name.append('joint_head_pan')
                goal.position.append(pos + 0.1)
            elif name == 'joint_head_tilt':
                goal.name.append('joint_head_tilt')
                goal.position.append(pos + 0.1)
            elif name == 'joint_gripper_finger_left':
                goal.name.append('joint_gripper')
                goal.position.append(pos + 0.1)
        joint_pose_publisher.publish_joint_pose(goal)
        joint_pose_publisher.lt.update()
        elp = time.time()-start
        if elp<1/target_rate:
            time.sleep((1/target_rate)-elp)


    i = 0
    joint_pose_publisher.lt.reset()
    while i<100:
        start = time.time()
        i = 1 + i
        rclpy.spin_once(joint_pose_publisher)
        j_state = joint_pose_publisher.joint_state
        print(j_state)
        goal = JointState()
        for name, pos in zip(j_state.name, j_state.position):
            if name == 'joint_lift':
                goal.name.append('joint_lift')
                goal.position.append(pos - 0.05)
            elif name == "wrist_extension":
                goal.name.append('joint_arm')
                goal.position.append(pos - 0.05)
            elif name == 'joint_wrist_pitch':
                goal.name.append('joint_wrist_pitch')
                goal.position.append(pos - 0.1)
            elif name == 'joint_wrist_roll':
                goal.name.append('joint_wrist_roll')
                goal.position.append(pos - 0.1)
            elif name == 'joint_wrist_yaw':
                goal.name.append('joint_wrist_yaw')
                goal.position.append(pos - 0.1)
            elif name == 'joint_head_pan':
                goal.name.append('joint_head_pan')
                goal.position.append(pos - 0.1)
            elif name == 'joint_head_tilt':
                goal.name.append('joint_head_tilt')
                goal.position.append(pos - 0.1)
            elif name == 'joint_gripper_finger_left':
                goal.name.append('joint_gripper')
                goal.position.append(pos - 0.1)
        joint_pose_publisher.publish_joint_pose(goal)
        joint_pose_publisher.lt.update()
        elp = time.time()-start
        if elp<1/target_rate:
            time.sleep((1/target_rate)-elp)



    joint_pose_publisher.destroy_node()
    rclpy.shutdown()