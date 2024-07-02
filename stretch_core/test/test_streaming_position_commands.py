import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from hello_helpers.joint_qpos_conversion import get_Idx
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import time

ROS_ARM_JOINTS = ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]
ROS_LIFT_JOINT = "joint_lift"
ROS_GRIPPER_FINGER = "joint_gripper_finger_left"
# ROS_GRIPPER_FINGER2 = "joint_gripper_finger_right"
ROS_HEAD_PAN = "joint_head_pan"
ROS_HEAD_TILT = "joint_head_tilt"
ROS_WRIST_YAW = "joint_wrist_yaw"
ROS_WRIST_PITCH = "joint_wrist_pitch"
ROS_WRIST_ROLL = "joint_wrist_roll"

class JointPosePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('float_array_publisher')
        self.reentrant_cb = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_pose_cmd', 10,callback_group=self.reentrant_cb)
        # subscribe to joint states
        self.joint_state = JointState()
        self.joint_states_subscriber = self.create_subscription(JointState, 
                                                                '/stretch/joint_states', 
                                                                self.joint_states_callback, 10,callback_group=self.reentrant_cb)
        self.Idx = get_Idx('eoa_wrist_dw3_tool_sg3')
        self.switch_to_position_mode_service = self.create_client(Trigger, '/switch_to_position_mode',callback_group=self.reentrant_cb)
        while not self.switch_to_position_mode_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/switch_to_position_mode' service...")
    
    def joint_states_callback(self, msg):
        self.joint_state = msg

    def switch_to_position_mode(self):
        trigger_request = Trigger.Request()
        trigger_result = self.switch_to_position_mode_service.call_async(trigger_request)
        return
        # while not trigger_result.done():
        #     time.sleep(0.2)
        # return trigger_result.done()
    
    def get_joint_status(self):
        j_status =  self.parse_joint_state(self.joint_state)
        pose = np.zeros(self.Idx.num_joints)
        pose[self.Idx.LIFT] = j_status[ROS_LIFT_JOINT]
        pose[self.Idx.ARM] = j_status[ROS_ARM_JOINTS[0]]+j_status[ROS_ARM_JOINTS[1]]+j_status[ROS_ARM_JOINTS[2]]+j_status[ROS_ARM_JOINTS[3]]
        pose[self.Idx.GRIPPER] = j_status[ROS_GRIPPER_FINGER]
        pose[self.Idx.WRIST_ROLL] = j_status[ROS_WRIST_ROLL]
        pose[self.Idx.WRIST_PITCH] = j_status[ROS_WRIST_PITCH]
        pose[self.Idx.WRIST_YAW] = j_status[ROS_WRIST_YAW]
        pose[self.Idx.HEAD_PAN] = j_status[ROS_HEAD_PAN]
        pose[self.Idx.HEAD_TILT] = j_status[ROS_HEAD_TILT]
        return pose
        
    def parse_joint_state(self, joint_state_msg):
        joint_status = {}
        for name, position in zip(joint_state_msg.name, joint_state_msg.position):
            joint_status[name] = position
        return joint_status

    def publish_joint_pose(self, joint_pose):
        msg = Float64MultiArray()
        msg.data = list(joint_pose)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

if __name__ == '__main__':
    joint_pose_publisher = JointPosePublisher()
    Idx = get_Idx('eoa_wrist_dw3_tool_sg3')
    joint_pose_publisher.switch_to_position_mode()
    i = 0
    while i<100:
        i = 1 + i
        rclpy.spin_once(joint_pose_publisher)
        qpos = joint_pose_publisher.get_joint_status()
        qpos[Idx.LIFT] = qpos[Idx.LIFT] + 0.05
        qpos[Idx.ARM] = qpos[Idx.ARM] + 0.05
        qpos[Idx.WRIST_PITCH] = qpos[Idx.WRIST_PITCH] + 0.1
        qpos[Idx.WRIST_ROLL] = qpos[Idx.WRIST_ROLL] + 0.1
        qpos[Idx.WRIST_YAW] = qpos[Idx.WRIST_YAW] + 0.1
        qpos[Idx.GRIPPER] = qpos[Idx.GRIPPER] + 10
        qpos[Idx.BASE_TRANSLATE] = 0.01
        qpos[Idx.BASE_ROTATE] = 0.0
        joint_pose_publisher.publish_joint_pose(qpos)
        time.sleep(1/15)
    i = 0
    while i<100:
        i = 1 + i
        rclpy.spin_once(joint_pose_publisher)
        qpos = joint_pose_publisher.get_joint_status()
        qpos[Idx.LIFT] = qpos[Idx.LIFT] - 0.05
        qpos[Idx.ARM] = qpos[Idx.ARM] - 0.05
        qpos[Idx.WRIST_PITCH] = qpos[Idx.WRIST_PITCH] - 0.1
        qpos[Idx.WRIST_ROLL] = qpos[Idx.WRIST_ROLL] - 0.1
        qpos[Idx.WRIST_YAW] = qpos[Idx.WRIST_YAW] - 0.1
        qpos[Idx.GRIPPER] = qpos[Idx.GRIPPER] - -10
        qpos[Idx.BASE_ROTATE] = -0.05
        qpos[Idx.BASE_TRANSLATE] = 0.0
        joint_pose_publisher.publish_joint_pose(qpos)
        time.sleep(1/15)

    joint_pose_publisher.destroy_node()
    rclpy.shutdown()