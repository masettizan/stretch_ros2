#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

import time
import threading
import numpy as np
import os
from math import pi

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


class PickupObjectNode (hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'pickup_obj', 'pickup_obj', wait_for_first_pointcloud=False)

        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.tool = None
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None
        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.aruco_marker_sub = self.create_subscription(MarkerArray, '/aruco/marker_array', self.aruco_marker_cb, 1)

        
        self.tf_buffer = tf2_ros.Buffer()
        self.obj_id = 50
        self.object_pose = None
        self.object_tf = None

        # Provide the min and max joint positions for the head pan. These values
        # are needed for sweeping the head to search for the ArUco tag
        self.min_pan_position = -3.8
        self.max_pan_position =  1.50

        # Define the number of steps for the sweep, then create the step size for
        # the head pan joint
        self.pan_num_steps = 10
        self.pan_step_size = abs(self.min_pan_position - self.max_pan_position)/self.pan_num_steps

        # Define the min tilt position, number of steps, and step size
        self.min_tilt_position = -1.0
        self.tilt_num_steps = 1
        self.tilt_step_size = pi/16

        # Define the head actuation rotational velocity
        self.rot_vel = 0.5 # radians per sec
        
        
    def joint_states_callback(self, joint_states):
        # with self.joint_states_lock: 
        self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position
        self.left_finger_position, temp1, temp2 = hm.get_left_finger_state(joint_states)




    def aruco_marker_cb(self, aruco_marker_array):
        # Look for the object id and set the object pose to the aruco tag pose
        # print(aruco_marker_array)
        for aruco_marker in aruco_marker_array.markers:
            
            if aruco_marker.id == self.obj_id:
                self.object_pose = aruco_marker.pose
                try:
                    self.object_tf = self.tf_buffer.lookup_transform('base_link', 'unknown', Time())
                except TransformException as ex:
                    continue
                break

    def main(self):
        self.trigger_head_scan_service = self.create_client(Trigger, '/funmap/trigger_head_scan')
        self.trigger_head_scan_service.wait_for_service()
        self.get_logger().info('Node ' + self.get_name() + ' connected to /funmap/trigger_head_scan.')

        # trigger_request = Trigger.Request() 
        # trigger_result = self.trigger_head_scan_service.call_async(trigger_request)
        self.find_tag(tag_name='unknown')

        self.get_logger().info("Found object at " 
                               + str(self.object_pose.position.x) + ", " 
                               + str(self.object_pose.position.y) + ", " 
                               + str(self.object_pose.position.z))
        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #     # Do nothing if object has not been found
        #     if self.object_pose is None:
        #         continue

        #     # Object has been detected


        self.destroy_node()
        rclpy.shutdown()


    def send_command(self, command):
        '''
        Handles single joint control commands by constructing a FollowJointTrajectoryGoal
        message and sending it to the trajectory_client created in hello_misc.
        :param self: The self reference.
        :param command: A dictionary message type.
        '''
        if (self.joint_states is not None) and (command is not None):

            # Extract the string value from the `joint` key
            joint_name = command['joint']

            # Set trajectory_goal as a FollowJointTrajectory.Goal and define
            # the joint name
            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [joint_name]

            # Create a JointTrajectoryPoint message type
            point = JointTrajectoryPoint()

            # Check to see if `delta` is a key in the command dictionary
            if 'delta' in command:
                # Get the current position of the joint and add the delta as a
                # new position value
                joint_index = self.joint_states.name.index(joint_name)
                joint_value = self.joint_states.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point.positions = [new_value]

            # Check to see if `position` is a key in the command dictionary
            elif 'position' in command:
                # extract the head position value from the `position` key
                point.positions = [command['position']]

            # Set the rotational velocity
            point.velocities = [self.rot_vel]

            # Assign goal position with updated point variable
            trajectory_goal.trajectory.points = [point]

            # Specify the coordinate frame that we want (base_link) and set the time to be now.
            trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory_goal.trajectory.header.frame_id = 'base_link'
            
            # Make the action call and send the goal. The last line of code waits
            # for the result
            self.trajectory_client.send_goal(trajectory_goal)

    def find_tag(self, tag_name='docking_station'):
        """
        A function that actuates the camera to search for a defined ArUco tag
        marker. Then the function returns the pose.
        :param self: The self reference.
        :param tag_name: A string value of the ArUco marker name.

        :returns transform: The docking station's TransformStamped message.
        """
        self.get_logger().info("Searching for the requested tag " + tag_name)
        # Create dictionaries to get the head in its initial position
        pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
        self.send_command(pan_command)
        tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
        self.send_command(tilt_command)

        # Nested for loop to sweep the joint_head_pan and joint_head_tilt in increments
        for i in range(self.tilt_num_steps):
            for j in range(self.pan_num_steps):
                # Update the joint_head_pan position by the pan_step_size
                pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
                self.send_command(pan_command)

                # Give time for system to do a Transform lookup before next step
                time.sleep(0.2)

                # Use a try-except block
                try:
                    # Look up transform between the base_link and requested ArUco tag
                    transform = self.tf_buffer.lookup_transform('base_link',
                                                            tag_name,
                                                            Time())
                    self.get_logger().info("Found Requested Tag: \n%s", transform)

                    # Publish the transform
                    self.object_tf = transform

                    # Return the transform
                    self.get_logger().info("Found tag %s" % tag_name)
                    return transform
                except TransformException as ex:
                    continue

            # Begin sweep with new tilt angle
            pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
            self.send_command(pan_command)
            tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
            self.send_command(tilt_command)
            time.sleep(0.25)

        # Notify that the requested tag was not found
        self.get_logger().info("The requested tag '%s' was not found" % tag_name)



def main():
    try:
        node = PickupObjectNode()
        node.main()
        
    except KeyboardInterrupt:
        rclpy.logging.get_logger('grasp_object').info('interrupt received, so shutting down')

if __name__ == '__main__':
    main()