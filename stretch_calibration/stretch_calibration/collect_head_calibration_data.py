#!/usr/bin/env python3

import argparse as ap
import math
import sys
import threading
import time

import message_filters
import numpy as np
import rclpy
import ros2_numpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray


class CollectHeadCalibrationDataNode(Node):
    def __init__(self):

        self.rate = 10.0

        self.joint_state = None
        self.acceleration = None

        self.data_time = None
        self.marker_time = None
        self.data_lock = threading.Lock()

    def calibration_data_callback(self, joint_state, accel, marker_array):
        # Prepare and assign data for calibration to member
        # variables. The configuration of the robot's joints, the
        # D435i acceleration, and the 3D fiducial estimates made via
        # D435i imagery are expected to be synchronized for this
        # callback.
        with self.data_lock:
            self.data_time = joint_state.header.stamp
            self.joint_state = joint_state
            self.acceleration = [accel.linear_acceleration.x, accel.linear_acceleration.y, accel.linear_acceleration.z]

            self.wrist_inside_marker_pose = None
            self.wrist_top_marker_pose = None
            self.base_left_marker_pose = None
            self.base_right_marker_pose = None
            self.shoulder_marker_pose = None

            self.marker_time = None
            for marker in marker_array.markers:

                # set marker_time to the earliest marker time
                if self.marker_time is None:
                    self.marker_time = marker.header.stamp
                elif (marker.header.stamp.sec + marker.header.stamp.nanosec*pow(10, -9)) < (self.marker_time.sec + self.marker_time.nanosec*pow(10, -9)):
                    self.marker_time = marker.header.stamp
                
                if marker.id == self.wrist_inside_marker_id:
                    self.wrist_inside_marker_pose = marker.pose

                if marker.id == self.wrist_top_marker_id:
                    self.wrist_top_marker_pose = marker.pose

                if marker.id == self.base_left_marker_id:
                    self.base_left_marker_pose = marker.pose

                if marker.id == self.base_right_marker_id:
                    self.base_right_marker_pose = marker.pose

                if marker.id == self.shoulder_marker_id:
                    self.shoulder_marker_pose = marker.pose

    def move_to_pose(self, pose):
        # Prepare and send a goal pose to which the robot should move.
        
        joint_names = [key for key in pose]
        self.trajectory_goal.trajectory.joint_names = joint_names
        
        joint_positions = [pose[key] for key in joint_names]
        self.point.positions = joint_positions

        self.trajectory_goal.trajectory.points = [self.point]
        goal_future = self.trajectory_client.send_goal_async(self.trajectory_goal)
        while not goal_future.done():
            time.sleep(0.1)
            rclpy.spin_once(self)
        
    def get_samples(self, pan_angle_center, tilt_angle_center, wrist_extension_center, number_of_samples, first_move=False):
        # Collect N observations (samples) centered around the
        # provided head and wrist extension pose. If first_move is
        # true, it indicates that this is the first movement to a new
        # region for calibration, which can induce more vibrations and
        # thus benefit from more time to settle prior to collecting a
        # sample.
        
        head_motion_settle_time = Duration(seconds=1.0) #5.0
        first_pan_tilt_extra_settle_time = Duration(seconds=1.0)
        wait_before_each_sample_s = Duration(seconds=0.2)

        # generate pan and wrist backlash samples

        # tilt backlash
        tilt_backlash_transition = self.tilt_angle_backlash_transition_rad
        # Ensure that the head is clearly in one of the two head tilt
        # backlash regimes and not in the indeterminate transition
        # region.
        tilt_backlash_safety_margin = 0.08        
        tilt_backlash_looking_down_max = tilt_backlash_transition - tilt_backlash_safety_margin # looking down
        tilt_backlash_looking_up_min = tilt_backlash_transition + tilt_backlash_safety_margin # looking up
        
        tilt_angle = tilt_angle_center
        if (tilt_angle_center > tilt_backlash_looking_down_max) and (tilt_angle_center < tilt_backlash_looking_up_min):
            # Commanded tilt_angle_center is in an uncertain backlash range
            # of tilt, so change it to be in a confident range.
            down_diff = abs(tilt_backlash_looking_down_max - tilt_angle_center)
            up_diff = abs(tilt_backlash_looking_up_min - tilt_angle_center)
            if up_diff < down_diff:
                tilt_angle = tilt_backlash_looking_up_min
            else:
                tilt_angle = tilt_backlash_looking_down_max
        if tilt_angle > tilt_backlash_transition: 
            joint_head_tilt_looking_up = True
        else:
            joint_head_tilt_looking_up = False
            
        # represent all head pan and wrist extension backlash states
        backlash_combinations = ((False, False), (False, True), (True, True), (True, False))

        pan_backlash_change_rad = 0.07 # ~8 degrees
        wrist_backlash_change_m = 0.01 # 1 cm
        
        samples = []
        
        # collect samples for all head pan and wrist extension
        # backlash states
        for joint_head_pan_looked_left, wrist_extension_retracted in backlash_combinations: 

            if joint_head_pan_looked_left:
                pan_angle = pan_angle_center + pan_backlash_change_rad
            else:
                pan_angle = pan_angle_center - pan_backlash_change_rad

            if wrist_extension_retracted: 
                wrist_extension = wrist_extension_center - wrist_backlash_change_m
            else:
                wrist_extension = wrist_extension_center + wrist_backlash_change_m

            # move to backlash state
            head_arm_pose = {'joint_head_pan': pan_angle,
                             'joint_head_tilt': tilt_angle,
                             'wrist_extension': wrist_extension}
            self.move_to_pose(head_arm_pose)


            # wait for the joints and sensors to settle
            self.clock.sleep_for(rel_time=head_motion_settle_time)
            settled_time = self.get_clock().now().to_msg()
            
            for i in range(5):
                rclpy.spin_once(self)

            # The first move to a pose is typically larger and can
            # benefit from more settling time due to induced
            # vibrations.
            if first_move:
                self.clock.sleep_for(rel_time=first_pan_tilt_extra_settle_time)

            for sample_number in range(number_of_samples):
                self.clock.sleep_for(rel_time=wait_before_each_sample_s)
                observation = {'joints': {'joint_head_pan': None,
                                          'joint_head_tilt': None,
                                          'wrist_extension' : None,
                                          'joint_lift': None },
                               'backlash_state': {
                                   'joint_head_pan_looked_left': None,
                                   'joint_head_tilt_looking_up': None,
                                   'wrist_extension_retracted': None
                                   },
                               'camera_measurements': { 'd435i_acceleration': None,
                                                        'wrist_inside_marker_pose': None,
                                                        'wrist_top_marker_pose': None,
                                                        'base_left_marker_pose': None,
                                                        'base_right_marker_pose': None,
                                                        'shoulder_marker_pose': None}
                               }

                # wait for a sample taken after the robot has
                # settled
                duration = Duration(seconds=10.0)
                timeout_duration = duration.to_msg()
                start_wait = self.get_clock().now().to_msg()
                timeout = False
                data_ready = False
                
                for i in range(50):
                    rclpy.spin_once(self)

                while (not data_ready) and (not timeout):
                    # check the timing of the current sample

                    for i in range(50):
                        rclpy.spin_once(self)

                    with self.data_lock:
                        if (self.data_time is not None) and ((self.data_time.sec + self.data_time.nanosec*pow(10,-9)) > (settled_time.sec + settled_time.nanosec*pow(10,-9))):
                            if (self.marker_time is None):
                                # joint_states and acceleration
                                # were taken after the robot
                                # settled and there are no aruco
                                # marker poses
                                data_ready = True
                            elif (self.marker_time.sec + self.marker_time.nanosec*pow(10,-9)) > (settled_time.sec + settled_time.nanosec*pow(10,-9)):
                                # joint_states, acceleration,
                                # and aruco marker poses were
                                # taken after the robot
                                # settled
                                data_ready = True
                    if not data_ready:
                        #rospy.sleep(0.2) #0.1
                        self.clock.sleep_for(Duration(seconds=1.0))
                    
                    current_time = self.get_clock().now().to_msg()
                    time_duration = (current_time.sec + current_time.nanosec*pow(10,-9)) - (start_wait.sec + start_wait.nanosec*pow(10,-9))
                    timeout = time_duration > timeout_duration.sec

                if timeout:
                    self.get_logger().error('collect_head_calibration_data get_samples: timeout while waiting for sample.')
                    raise Exception('Timed out waiting for joint_states/accelerations/markers messages after 10 seconds')

                with self.data_lock:
                    #set backlash joint state
                    backlash_state = observation['backlash_state']
                    backlash_state['joint_head_pan_looked_left'] = joint_head_pan_looked_left
                    backlash_state['joint_head_tilt_looking_up'] = joint_head_tilt_looking_up
                    backlash_state['wrist_extension_retracted'] = wrist_extension_retracted

                    # record the settled joint values
                    joints = observation['joints']
                    for joint_name in joints:
                        print('Joint name %s'%joint_name)
                        joint_index = self.joint_state.name.index(joint_name)
                        joint_value = self.joint_state.position[joint_index]
                        joints[joint_name] = joint_value

                    # record the settled camera measurments
                    camera_measurements = observation['camera_measurements']
                    camera_measurements['d435i_acceleration'] = self.acceleration

                    # record the settled aruco measurements
                    if self.wrist_inside_marker_pose is not None:
                        camera_measurements['wrist_inside_marker_pose'] = ros2_numpy.numpify(self.wrist_inside_marker_pose).tolist()

                    if self.wrist_top_marker_pose is not None:
                        camera_measurements['wrist_top_marker_pose'] = ros2_numpy.numpify(self.wrist_top_marker_pose).tolist()

                    if self.base_left_marker_pose is not None:
                        camera_measurements['base_left_marker_pose'] = ros2_numpy.numpify(self.base_left_marker_pose).tolist()

                    if self.base_right_marker_pose is not None:
                        camera_measurements['base_right_marker_pose'] = ros2_numpy.numpify(self.base_right_marker_pose).tolist()

                    if self.shoulder_marker_pose is not None:
                        camera_measurements['shoulder_marker_pose'] = ros2_numpy.numpify(self.shoulder_marker_pose).tolist()

                samples.append(observation)
        return samples

    def calibrate_pan_and_tilt(self, collect_check_data=False):
        # Collects observations of fiducial markers on the robot's
        # body while moving the head pan, head tilt, arm lift, and arm
        # extension. There are five markers in total with the
        # following locations: front left of the mobile base, front
        # right of the mobile base, top of the shoulder, top of the
        # wrist, and inside the wrist.
        #
        # When collect_check_data is True, fewer samples are collected
        # with the intention of using them to check the current
        # calibration. When collect_check_data is False, more samples
        # are collected with the intention of fully calibrating the
        # robot.
        
        calibration_data = []
        number_of_samples_per_head_pose = 1 #3
        wrist_motion_settle_secs = 1.0
        wrist_motion_settle_time = Duration(seconds=wrist_motion_settle_secs)

        ########################################
        ##
        ## COLLECT GLOBAL HEAD LOOKING DOWN DATA
        ##
        ########################################
        self.get_logger().info('')
        self.get_logger().info('*************************************')
        self.get_logger().info('COLLECT GLOBAL HEAD LOOKING DOWN DATA')
        self.get_logger().info('*************************************')
        self.get_logger().info('')

        # looking down data
        min_tilt_angle_rad = -1.4
        max_tilt_angle_rad = -0.5 

        min_pan_angle_rad = -3.8
        max_pan_angle_rad = 1.3

        if collect_check_data:
            number_of_tilt_steps = 1 
            number_of_pan_steps = 3 
        else: 
            number_of_tilt_steps = 3 #4 
            number_of_pan_steps = 5 #7 

        pan_angles_rad = np.linspace(min_pan_angle_rad, max_pan_angle_rad, number_of_pan_steps)
        tilt_angles_rad = np.linspace(min_tilt_angle_rad, max_tilt_angle_rad, number_of_tilt_steps)

        self.get_logger().info('Move to a new arm pose.')
        wrist_extension = 0.12
        wrist_pose = {'wrist_extension': wrist_extension}
        self.move_to_pose(wrist_pose)
        lift_m = 0.16
        lift_pose = {'joint_lift': lift_m}
        self.move_to_pose(lift_pose)

        # wait for the joints and sensors to settle
        self.get_logger().info('Wait {0} seconds for the robot to settle after motion of the arm.'.format(wrist_motion_settle_secs))
        self.clock.sleep_for(rel_time=wrist_motion_settle_time)

        self.get_logger().info('Starting to collect global head looking down data  (expect to collect {0} samples).'.format(number_of_tilt_steps * number_of_pan_steps))
        first_pan_tilt = True
        for pan_angle in pan_angles_rad:
            for tilt_angle in tilt_angles_rad:
                observation = self.get_samples(pan_angle, tilt_angle, wrist_extension, number_of_samples_per_head_pose, first_move=first_pan_tilt)
                first_pan_tilt = False
                calibration_data.extend(observation)
                n = len(calibration_data)
                if (n % 10) == 0:
                    self.get_logger().info('{0} samples collected so far.'.format(n))

        #######################################
        ##
        ## COLLECT GLOBAL HEAD LOOKING UP DATA
        ##
        #######################################
        self.get_logger().info('')
        self.get_logger().info('*************************************')
        self.get_logger().info('COLLECT GLOBAL HEAD LOOKING UP DATA')
        self.get_logger().info('*************************************')
        self.get_logger().info('')
        
        if not collect_check_data:
            tilt_angles_rad = [-0.3, 0.38]
            pan_angles_rad = [-3.8, -1.66, 1.33]

            self.get_logger().info('Move to a new arm pose.')
            lift_m = 0.92
            lift_pose = {'joint_lift': lift_m}
            self.move_to_pose(lift_pose)
            wrist_extension = 0.29
            wrist_pose = {'wrist_extension': wrist_extension}
            self.move_to_pose(wrist_pose)

            # wait for the joints and sensors to settle
            self.get_logger().info('Wait {0} seconds for the robot to settle after motion of the arm.'.format(wrist_motion_settle_secs))
            self.clock.sleep_for(rel_time=wrist_motion_settle_time)

            self.get_logger().info('Starting to collect global head looking up data (expect to collect {0} samples).'.format(len(pan_angles_rad) * len(tilt_angles_rad)))
            first_pan_tilt = True
            for pan_angle in pan_angles_rad:
                for tilt_angle in tilt_angles_rad:
                    observation = self.get_samples(pan_angle, tilt_angle, wrist_extension, number_of_samples_per_head_pose, first_move=first_pan_tilt)
                    first_pan_tilt = False
                    calibration_data.extend(observation)
                    n = len(calibration_data)
                    if (n % 10) == 0:
                        self.get_logger().info('{0} samples collected so far.'.format(n))
                    
        #######################################
        ##
        ## COLLECT HIGH SHOULDER DATA
        ##
        #######################################
        self.get_logger().info('')
        self.get_logger().info('*************************************')
        self.get_logger().info('COLLECT HIGH SHOULDER DATA')
        self.get_logger().info('*************************************')
        self.get_logger().info('')
        
        ##############################################################
        ### poses that see the shoulder marker
        #
        # lift 0.8
        # pan  -3.82 to -3.619
        # tilt -0.918 to -0.49
        #
        # lift 0.2
        # pan  -3.85 to -2.48
        # tilt -1.23 to -0.87
        #
        # two poses with good coverage
        # (needs to account for +/- 0.07 rad for pan backlash)
        # pan -3.7, tilt -0.7
        # pan -3.7, tilt -1.1
        #
        # Marker detected with point clouds
        # lift 0.5,  pan -3.7,  tilt = -1.4 to -1.2
        # lift 0.65, pan -3.85, tilt = -1.0
        # lift 0.74, pan -3.85, tilt = -0.78 to -1.4 (-1.4 also sees the base markers)
        ##############################################################

        wrist_extension = 0.29
        
        # low shoulder height
        pan_angle_rad = -3.7
        tilt_angle_rad_1 = -1.4
        tilt_angle_rad_2 = -1.2
        lift_m = 0.5

        pose = {'joint_lift': lift_m}
        self.get_logger().info('Move to first shoulder capture height.')
        self.move_to_pose(pose)
        self.get_logger().info('Wait {0} seconds for the robot to settle after motion of the arm.'.format(wrist_motion_settle_secs))
        self.clock.sleep_for(rel_time=wrist_motion_settle_time)

        first_pan_tilt = True
        observation = self.get_samples(pan_angle_rad, tilt_angle_rad_1, wrist_extension, number_of_samples_per_head_pose, first_move=first_pan_tilt)
        calibration_data.extend(observation)

        if not collect_check_data:
            first_pan_tilt = False
            observation = self.get_samples(pan_angle_rad, tilt_angle_rad_2, wrist_extension, number_of_samples_per_head_pose, first_move=first_pan_tilt)
            calibration_data.extend(observation)

        # high shoulder height
        pan_angle_rad = -3.85
        tilt_angle_rad_1 = -0.8
        tilt_angle_rad_2 = -1.4
        lift_m = 0.74

        pose = {'joint_lift': lift_m}
        self.get_logger().info('Move to second shoulder capture height.')
        self.move_to_pose(pose)
        self.get_logger().info('Wait {0} seconds for the robot to settle after motion of the arm.'.format(wrist_motion_settle_secs))
        self.clock.sleep_for(rel_time=wrist_motion_settle_time)

        first_pan_tilt = True
        observation = self.get_samples(pan_angle_rad, tilt_angle_rad_1, wrist_extension, number_of_samples_per_head_pose, first_move=first_pan_tilt)
        calibration_data.extend(observation)

        if not collect_check_data:
            first_pan_tilt = False
            observation = self.get_samples(pan_angle_rad, tilt_angle_rad_2, wrist_extension, number_of_samples_per_head_pose, first_move=first_pan_tilt)
            calibration_data.extend(observation)


        #######################################
        ##
        ## COLLECT ARM FOCUSED DATA
        ##
        #######################################
        self.get_logger().info('')
        self.get_logger().info('*************************************')
        self.get_logger().info('COLLECT ARM FOCUSED DATA')
        self.get_logger().info('*************************************')
        self.get_logger().info('')

        ##############################################################
        ### poses that see both of the arm markers
        #
        # lift height 0.8
        # extension 0.4   tilt -0.52    pan -2.06 to -1.45
        # extension 0.03  tilt -1.27    pan -2.33 to -1.48
        #
        # lift height 0.2
        # extension 0.4   tilt -0.9     pan -2.9 to -0.93
        # extension 0.03  tilt -1.23    pan -3.2 to -0.4 (full range)
        #
        # ranges with good coverage
        # (needs to account for +/- 0.07 rad for pan backlash)
        # tilt -1.2 to -0.5
        # pan  -2.0 to -1.45
        #
        ##############################################################

        wrist_focused_min_pan = -2.0
        wrist_focused_max_pan = -1.45

        wrist_focused_min_tilt = -1.2
        wrist_focused_max_tilt = -0.5

        
        if collect_check_data:
            num_wrist_focused_pan_steps = 1 #3
            num_wrist_focused_tilt_steps = 1 #3
        else:
            num_wrist_focused_pan_steps = 2 #3
            num_wrist_focused_tilt_steps = 2 #3
            
        wrist_focused_pan_angles_rad = np.linspace(wrist_focused_min_pan, wrist_focused_max_pan, num_wrist_focused_pan_steps)
        wrist_focused_tilt_angles_rad = np.linspace(wrist_focused_min_tilt, wrist_focused_max_tilt, num_wrist_focused_tilt_steps)

        if collect_check_data:
            wrist_extensions = [0.2]
            # Already have data from a single extension at a height of
            # 1.2m from global camera looking down, so try using only 2
            # heights.
            #
            # 0.46 m = (0.8 m + 0.12 m) / 2.0
            wrist_heights = [0.46, 0.8] #[0.2, 0.5, 0.8]
            wrist_poses = [{'wrist_extension': e, 'joint_lift': h} for h in wrist_heights for e in wrist_extensions] 

            num_wrist_poses = len(wrist_poses)
        else:
            wrist_extensions = [0.03, 0.4]
            # Already have data from a single extension at a height of
            # 1.2m from global camera looking down, so try using only 2
            # heights.
            #
            # 0.46 m = (0.8 m + 0.12 m) / 2.0
            wrist_heights = [0.46, 0.8] #[0.2, 0.5, 0.8]
            wrist_poses = [{'wrist_extension': e, 'joint_lift': h} for h in wrist_heights for e in wrist_extensions] 

            num_wrist_poses = len(wrist_poses)

        self.get_logger().info('Starting to collect arm focused samples (expect to collect {0} samples).'.format(num_wrist_poses * num_wrist_focused_pan_steps * num_wrist_focused_tilt_steps))

        for wrist_pose in wrist_poses:

            self.get_logger().info('Move to a new arm pose.')
            self.move_to_pose(wrist_pose)

            # wait for the joints and sensors to settle
            self.get_logger().info('Wait {0} seconds for the robot to settle after motion of the arm.'.format(wrist_motion_settle_secs))
            self.clock.sleep_for(rel_time=wrist_motion_settle_time)

            first_pan_tilt = True
            for pan_angle in wrist_focused_pan_angles_rad:
                for tilt_angle in wrist_focused_tilt_angles_rad:
                    observation = self.get_samples(pan_angle, tilt_angle, wrist_pose['wrist_extension'], number_of_samples_per_head_pose, first_move=first_pan_tilt)
                    first_pan_tilt = False
                    calibration_data.extend(observation)
                    n = len(calibration_data)
                    if (n % 10) == 0:
                        self.get_logger().info('{0} samples collected so far.'.format(n))
        
        #######################################
        ##
        ## FINISHED COLLECTING DATA - SAVE IT
        ##
        #######################################
        self.get_logger().info('')
        self.get_logger().info('*************************************')
        self.get_logger().info('FINISHED COLLECTING DATA')
        self.get_logger().info('*************************************')
        self.get_logger().info('')
        self.get_logger().info('Collected {0} samples in total.'.format(len(calibration_data)))
        
        t = time.localtime()
        capture_date = str(t.tm_year) + str(t.tm_mon).zfill(2) + str(t.tm_mday).zfill(2) + str(t.tm_hour).zfill(2) + str(t.tm_min).zfill(2)
        if collect_check_data:
            filename = self.calibration_directory + 'check_head_calibration_data_' + capture_date + '.yaml'
        else:
            filename = self.calibration_directory + 'head_calibration_data_' + capture_date + '.yaml'
        self.get_logger().info('Saving calibration_data to a YAML file named {0}'.format(filename))
        fid = open(filename, 'w')
        yaml.dump(calibration_data, fid)
        fid.close()
        self.get_logger().info('')
        self.get_logger().info('*************************************')
        self.get_logger().info('')

        #######################################
            
    def move_to_initial_configuration(self):
        # The robot is commanded to move to this pose prior to
        # beginning to collect calibration data.
        initial_pose = {'joint_wrist_yaw': 0.0,
                        'wrist_extension': 0.0,
                        'joint_lift': 0.3,
                        # 'gripper_aperture': 0.0, // TODO: check what parameter this is 
                        'joint_head_pan': -1.6947147036864942,
                        'joint_head_tilt': -0.4}

        self.get_logger().info('Move to the calibration start pose.')
        self.move_to_pose(initial_pose)
   
    def main(self, collect_check_data):
        rclpy.init()
        super().__init__('collect_head_calibration_data',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        self.node_name = self.get_name()        
        self.get_logger().info("{0} started".format(self.node_name))

        self.clock = Clock()

        # Obtain the ArUco marker ID numbers.
        # Reading parameters from the stretch_marker_dict.yaml file and storing values
        # in a dictionary called marker_info
        param_list = ['130', '131', '132', '133', '134', '246', '247', '248', '249', '10', '21', 'default']
        key_list = ['length_mm', 'use_rgb_only', 'name', 'link']
        dict = {}
        self.marker_info = {}
        for aruco_id in param_list:
            for key in key_list:
                dict[key] = self.get_parameter_or('aruco_marker_info.{0}.{1}'.format(aruco_id, key)).value
            self.marker_info[aruco_id] = dict
            dict = {}

        for k in self.marker_info.keys():
            m = self.marker_info[k]
            if m['link'] == 'link_aruco_left_base':
                self.base_left_marker_id = int(k)
            if m['link'] == 'link_aruco_right_base':
                self.base_right_marker_id = int(k)
            if m['link'] == 'link_aruco_inner_wrist':
                self.wrist_inside_marker_id = int(k)
            if m['link'] == 'link_aruco_top_wrist':
                self.wrist_top_marker_id = int(k)
            if m['link'] == 'link_aruco_shoulder':
                self.shoulder_marker_id = int(k)

        filename = self.get_parameter_or('controller_calibration_file').value

        self.get_logger().info('Loading factory default tilt backlash transition angle from the YAML file named {0}'.format(filename))
        fid = open(filename, 'r')
        controller_parameters = yaml.load(fid, Loader=yaml.SafeLoader)
        fid.close()
        self.tilt_angle_backlash_transition_rad = controller_parameters['tilt_angle_backlash_transition']
        deg_per_rad = 180.0/math.pi
        self.get_logger().info('self.tilt_angle_backlash_transition_rad in degrees = {0}'.format(self.tilt_angle_backlash_transition_rad * deg_per_rad))

        self.calibration_directory = self.get_parameter_or('calibration_directory').value
        self.get_logger().info('Using the following directory for calibration files: {0}'.format(self.calibration_directory))

        # Setup time synchronization for calibration data. 
        joint_state_subscriber = message_filters.Subscriber(self, JointState, '/stretch/joint_states')
        accel_subscriber = message_filters.Subscriber(self, Imu, '/camera/accel/sample_corrected')
        aruco_subscriber = message_filters.Subscriber(self, MarkerArray, '/aruco/marker_array')
        slop_time = 0.1
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([joint_state_subscriber, accel_subscriber, aruco_subscriber], 10, slop_time, allow_headerless=True)
        self.synchronizer.registerCallback(self.calibration_data_callback)

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()
        
        self.trajectory_goal = FollowJointTrajectory.Goal()
        goal_time_tolerance = Duration(seconds=1.0)
        self.trajectory_goal.goal_time_tolerance = goal_time_tolerance.to_msg()

        self.point = JointTrajectoryPoint()
        time_from_start = Duration(seconds=0.0)
        self.point.time_from_start = time_from_start.to_msg()

        # Spin to get current joint states, accel, marker_array
        while self.joint_state is None:
            rclpy.spin_once(self)
        
        self.move_to_initial_configuration()

        self.calibrate_pan_and_tilt(collect_check_data)


def main():
    parser = ap.ArgumentParser(description='Collect head calibration data.')
    parser.add_argument('--check', action='store_true', help='Collect data to check the current calibration, instead of data to perform a new calibration.')
    
    args, unknown = parser.parse_known_args()
    collect_check_data = args.check
     
    node = CollectHeadCalibrationDataNode()
    node.main(collect_check_data)

if __name__ == '__main__':    
    main()
