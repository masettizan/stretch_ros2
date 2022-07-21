#!/usr/bin/env python3

import calibration as ca

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Pose

import math
import time
import threading
import sys

import numpy as np
import ros2_numpy

from copy import deepcopy

import yaml
import glob
import argparse as ap

from urdf_parser_py.urdf import URDF as urdf_parser
import urdf_parser_py as up

from scipy.spatial.transform import Rotation
import cma


class HeadCalibrator:

    def __init__(self, node, uncalibrated_urdf_filename, calibration_directory, sample_selector_func, calibration_options, visualize, tilt_angle_backlash_transition_rad):
        self.node = node
        self.visualize = visualize
        self.infinite_duration_visualization = False

        self.observations_used_for_fit = {}
        
        self.telescoping_joints = ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']

        self.tilt_angle_backlash_transition_rad = tilt_angle_backlash_transition_rad
        
        self.calibrate_lift = calibration_options.get('calibrate_lift', False)
        self.calibrate_arm = calibration_options.get('calibrate_arm', False)
        self.calibrate_controller_offsets = calibration_options.get('calibrate_controller_offsets', False)

        self.calibrate_pan_backlash = calibration_options.get('calibrate_pan_backlash', False)
        self.calibrate_tilt_backlash = calibration_options.get('calibrate_tilt_backlash', False)
        self.calibrate_arm_backlash = calibration_options.get('calibrate_arm_backlash', False)
        self.default_backlash_state = {'joint_head_pan_looked_left':False, 'joint_head_tilt_looking_up':False, 'wrist_extension_retracted':False}
        
        self.use_this_sample = sample_selector_func 
        
        self.calibration_directory = calibration_directory
        self.head_calibration_data_filename = None

        if self.visualize: 
            self.visualization_markers_pub = self.node.create_publisher(MarkerArray, '/calibration/marker_array', queue_size=1)
        else:
            self.visualization_markers_pub = None

        self.pan_angle_offset = 0.0
        self.tilt_angle_offset = 0.0

        self.pan_looked_left_offset = 0.0
        self.tilt_looking_up_offset = 0.0
        self.arm_retracted_offset = 0.0

        self.initial_error_terms = {'wrist_top_pos': 0.0,
                                    'wrist_top_orient': 0.0,
                                    'wrist_inside_pos': 0.0,
                                    'wrist_inside_orient': 0.0,
                                    'shoulder_pos': 0.0,
                                    'shoulder_orient': 0.0, 
                                    'base_left_pos': 0.0,
                                    'base_left_orient': 0.0,
                                    'base_right_pos': 0.0,
                                    'base_right_orient': 0.0,
                                    'parameter_error_total': 0.0}

        self.error_weights = {'wrist_top_pos': 1.0,
                              'wrist_top_orient': 0.1,
                              'wrist_inside_pos': 1.0,
                              'wrist_inside_orient': 0.1,
                              'shoulder_pos': 0.5,
                              'shoulder_orient': 0.05, 
                              'base_left_pos': 0.5,
                              'base_left_orient': 0.05,
                              'base_right_pos': 0.5,
                              'base_right_orient': 0.05,
                              'joint_head_tilt_looking_up_error_multiplier': 2.0,
                              'parameter_error_total': 1.0}

        # Load and store the original uncalibrated URDF.
        self.original_urdf = urdf_parser.from_xml_file(uncalibrated_urdf_filename)

        # Initialize the new URDF that will altered during optimization.
        self.new_urdf = deepcopy(self.original_urdf)
        
        self.gravity_acceleration_vec = np.array([0.0, 0.0, 1.0])

        ##########
        # Set the kinematic chains and joints to be used during
        # optimization. These *_joint and *_chain variables are used
        # to access and modify the optimized URDF (self.new_urdf)
        # during optimization.

        # Also, set the original joint positions and rotations. The
        # optimization applies rigid body transformations to these
        # original poses.

        # Chain from the base of the robot to the color camera used to
        # observe the ARUCO markers.
        self.camera_chain = ca.Chain(self.new_urdf, 'base_link', 'camera_color_optical_frame')

        # This is the part that connects the pan joint to the top of
        # the mast. There is some variation in how it is mounted to
        # the top of the mast. For example, it can be rotated
        # slightly.

        self.camera_tilt_joint = self.camera_chain.get_joint_by_name('joint_head_tilt')
        self.original_tilt_assembly_xyz = np.array(self.camera_tilt_joint.origin.xyz)
        self.original_tilt_assembly_rpy = np.array(self.camera_tilt_joint.origin.rpy)
        self.original_tilt_assembly_r = Rotation.from_euler('xyz', self.original_tilt_assembly_rpy)
        
        self.camera_pan_joint = self.camera_chain.get_joint_by_name('joint_head_pan')
        self.original_pan_assembly_xyz = np.array(self.camera_pan_joint.origin.xyz)
        self.original_pan_assembly_rpy = np.array(self.camera_pan_joint.origin.rpy)
        self.original_pan_assembly_r = Rotation.from_euler('xyz', self.original_pan_assembly_rpy)

        self.camera_mount_joint = self.camera_chain.get_joint_by_name('camera_joint')
        self.original_camera_mount_xyz = np.array(self.camera_mount_joint.origin.xyz)
        self.original_camera_mount_rpy = np.array(self.camera_mount_joint.origin.rpy)
        self.original_camera_mount_r = Rotation.from_euler('xyz', self.original_camera_mount_rpy)

        # Chain from the base of the robot to robot's wrist that has ARUCO markers on it. 
        self.wrist_chain = ca.Chain(self.new_urdf, 'base_link', 'link_arm_l0')

        if self.calibrate_lift:
            # Calibrating the following joint ('joint_mast') results
            # in better visualizations, since it actually moves the
            # mast and keeps the carriage traveling along the
            # mast. Without constraints, this should permit the same
            # solutions as calibrating 'joint_lift'. With constraints,
            # the possible solutions can differ due to the mast also
            # influencing the position and orientation of the head
            # assembly.
            self.lift_joint = self.wrist_chain.get_joint_by_name('joint_mast')            
            self.original_lift_xyz = np.array(self.lift_joint.origin.xyz)
            self.original_lift_rpy = np.array(self.lift_joint.origin.rpy)
            self.original_lift_r = Rotation.from_euler('xyz', self.original_lift_rpy)

        if self.calibrate_arm:
            # This calibrates the position and orientation of the arm
            # with respect to the shoulder carriage that moves up and
            # down the lift. This compensates for variations in the
            # way the arm has been mounted.
            self.arm_joint = self.wrist_chain.get_joint_by_name('joint_arm_l4')
            self.original_arm_xyz = np.array(self.arm_joint.origin.xyz)
            self.original_arm_rpy = np.array(self.arm_joint.origin.rpy)
            self.original_arm_r = Rotation.from_euler('xyz', self.original_arm_rpy)

        ##########
            
        # calculate 10 deg of error
        deg_error = 10.0
        rad_error = 10.0 * (math.pi/180.0)
        calib_error = math.cos(rad_error)
        calib_error = (1.0 - calib_error) / 2.0

        # convert error to meters of error per degree
        self.meters_per_deg = 0.02 / calib_error # 2cm of error per 10 deg

        self.error_measures = []

        # Create error measures to be used during the optimization.
        aruco_urdf = self.new_urdf
        rgba = [0.0, 1.0, 0.0, 0.2]
        self.error_measures.append(ca.ArucoError('wrist_top', 'wrist', 'link_aruco_top_wrist', aruco_urdf, self.meters_per_deg, rgba))
        rgba = [0.0, 0.0, 1.0, 0.2]
        self.error_measures.append(ca.ArucoError('wrist_inside', 'wrist', 'link_aruco_inner_wrist', aruco_urdf, self.meters_per_deg, rgba))
        rgba = [1.0, 1.0, 0.0, 0.2]
        self.error_measures.append(ca.ArucoError('shoulder', 'shoulder', 'link_aruco_shoulder', aruco_urdf, self.meters_per_deg, rgba))
        rgba = [1.0, 0.0, 0.0, 0.2]
        self.error_measures.append(ca.ArucoError('base_left', 'base', 'link_aruco_left_base', aruco_urdf, self.meters_per_deg, rgba))
        rgba = [1.0, 0.0, 1.0, 0.2]
        self.error_measures.append(ca.ArucoError('base_right', 'base', 'link_aruco_right_base', aruco_urdf, self.meters_per_deg, rgba))
        
    def load_data(self, parameters, use_check_calibration_data=False):
        # Load data to use for calibration.

        if use_check_calibration_data:
            filenames = glob.glob(self.calibration_directory + 'check_head_calibration_data' + '_*[0-9].yaml')
        else: 
            filenames = glob.glob(self.calibration_directory + 'head_calibration_data' + '_*[0-9].yaml')
        filenames.sort()
        most_recent_filename = filenames[-1]
        self.head_calibration_data_filename = most_recent_filename
        print('Loading most recent head calibration data from a YAML file named ' + self.head_calibration_data_filename)
        fid = open(self.head_calibration_data_filename, 'r')
        self.data = yaml.load(fid)
        fid.close()

        # Convert data in yaml file from readable text to full joint
        # state representation and ROS Poses
        marker_prefixes = ['wrist_top', 'wrist_inside', 'shoulder', 'base_left', 'base_right']
            
        for i, s in enumerate(self.data):
            # Convert from wrist_extension to joint links for compatibility with the URDF
            joints = s['joints']
            # convert wrist_extension to telescoping joints for use with the URDF
            wrist_extension = joints.pop('wrist_extension')
            telescoping_value = wrist_extension/float(len(self.telescoping_joints))
            for j in self.telescoping_joints:
                joints[j] = telescoping_value

            # Convert from 4x4 homogeneous transformation matrices
            # represented as lists to ROS Poses
            camera_measurements = s['camera_measurements']
            for p in marker_prefixes:
                pose_key = p + '_marker_pose'
                pose_list = camera_measurements.get(pose_key)
                if pose_list is not None:
                    pose = ros2_numpy.msgify(Pose, np.array(pose_list))
                    camera_measurements[pose_key] = pose

        self.calculate_normalization()
        if self.visualize:
            self.update_urdf(parameters)
            self.target_markers = self.generate_target_visualizations()        
            marker_array = MarkerArray()
            marker_array.markers.extend(self.target_markers)
            self.visualization_markers_pub.publish(marker_array)

    def check_fit_error(self, fit_error):
        fit_error_threshold = 0.05
        fit_warning_threshold = 0.03
        if fit_error > fit_error_threshold:
            self.node.get_logger().error('The fit error is very high: {0} > {1} (fit_error > fit_error_threshold)'.format(fit_error, fit_error_threshold))
        elif fit_error > fit_warning_threshold:
            self.node.get_logger().warn('The fit error is high: {0} > {1} (fit_error > fit_warning_threshold)'.format(fit_error, fit_warning_threshold))
        

    def get_calibration_options(self):
        # Return the current calibration options
        
        calibration_options = {'calibrate_lift': self.calibrate_lift,
                               'calibrate_arm': self.calibrate_arm,
                               'calibrate_controller_offsets': self.calibrate_controller_offsets,
                               'calibrate_pan_backlash': self.calibrate_pan_backlash,
                               'calibrate_tilt_backlash': self.calibrate_tilt_backlash,
                               'calibrate_arm_backlash': self.calibrate_arm_backlash}
        return calibration_options
            
    def get_names_of_parameters_to_fit(self):
        # Return the names of the parameters being fit by the optimization.
        
        parameter_names = []
        
        if self.calibrate_controller_offsets:
            parameter_names.extend(['pan_angle_offset', 'tilt_angle_offset'])

        if self.calibrate_pan_backlash:
            parameter_names.extend(['pan_looked_left_offset'])

        if self.calibrate_tilt_backlash:
            parameter_names.extend(['tilt_looking_up_offset'])

        if self.calibrate_arm_backlash:
            parameter_names.extend(['arm_retracted_offset'])
        
        parameter_names.extend(['head_assembly_offset_xyz_x', 'head_assembly_offset_xyz_y', 'head_assembly_offset_xyz_z',
                                'head_assembly_offset_rotvec_x', 'head_assembly_offset_rotvec_y', 'head_assembly_offset_rotvec_z',
                                'pan_assembly_offset_xyz_x', 'pan_assembly_offset_xyz_y', 'pan_assembly_offset_xyz_z',
                                'pan_assembly_offset_rotvec_x', 'pan_assembly_offset_rotvec_y', 'pan_assembly_offset_rotvec_z',
                                'tilt_assembly_offset_xyz_x', 'tilt_assembly_offset_xyz_y', 'tilt_assembly_offset_xyz_z',
                                'tilt_assembly_offset_rotvec_x', 'tilt_assembly_offset_rotvec_y', 'tilt_assembly_offset_rotvec_z',
                                'camera_mount_offset_xyz_x', 'camera_mount_offset_xyz_y', 'camera_mount_offset_xyz_z',
                                'camera_mount_offset_rotvec_x', 'camera_mount_offset_rotvec_y', 'camera_mount_offset_rotvec_z'])
            
        if self.calibrate_lift:
            parameter_names.extend(['lift_offset_xyz_x', 'lift_offset_xyz_y', 'lift_offset_xyz_z',
                                    'lift_offset_rotvec_x', 'lift_offset_rotvec_y', 'lift_offset_rotvec_z'])
            
        if self.calibrate_arm:
            parameter_names.extend(['arm_offset_xyz_x', 'arm_offset_xyz_y', 'arm_offset_xyz_z',
                                    'arm_offset_rotvec_x', 'arm_offset_rotvec_y', 'arm_offset_rotvec_z'])

        return parameter_names

    def get_controller_parameters(self):
        return {'tilt_angle_offset': self.tilt_angle_offset,
                'pan_angle_offset': self.pan_angle_offset,
                'pan_looked_left_offset': self.pan_looked_left_offset,
                'tilt_looking_up_offset': self.tilt_looking_up_offset,
                'arm_retracted_offset': self.arm_retracted_offset,
                'tilt_angle_backlash_transition': self.tilt_angle_backlash_transition_rad}
    
    def parameter_error_term(self, parameters):
        # Calculate error that is solely a function of the parameter
        # values. This is used to keep parameters within desired
        # ranges by assigning high errors when parameters are outside
        # these ranges using a soft constraint function.

        ###########
        # Unpack and preprocess the parameters
        
        i = 0

        if self.calibrate_controller_offsets: 
            pan_angle_offset_abs = abs(parameters[i])
            i += 1
            tilt_angle_offset_abs = abs(parameters[i])
            i += 1

        if self.calibrate_pan_backlash:
            pan_looked_left_offset_abs = abs(parameters[i])
            i += 1
            
        if self.calibrate_tilt_backlash:
            tilt_looking_up_offset_abs = abs(parameters[i])
            i += 1
            
        if self.calibrate_arm_backlash:
            arm_retracted_offset_abs = abs(parameters[i]) 
            i += 1
            
        head_assembly_offset_xyz_length = np.linalg.norm(parameters[i:i+3])
        i += 3
        head_assembly_offset_rotvec_mag = np.linalg.norm(parameters[i:i+3])
        i += 3
            
        pan_assembly_offset_xyz_length = np.linalg.norm(parameters[i:i+3])
        i += 3
        pan_assembly_offset_rotvec_mag = np.linalg.norm(parameters[i:i+3])
        i += 3
        
        tilt_assembly_offset_xyz_length = np.linalg.norm(parameters[i:i+3])
        i += 3
        tilt_assembly_offset_rotvec_mag = np.linalg.norm(parameters[i:i+3])
        i += 3

        camera_mount_offset_xyz_length = np.linalg.norm(parameters[i:i+3])
        i += 3
        camera_mount_offset_rotvec_mag = np.linalg.norm(parameters[i:i+3])
        i += 3

        if self.calibrate_lift:
            lift_offset_xyz_length = np.linalg.norm(parameters[i:i+3])
            i += 3
            lift_offset_rotvec_mag = np.linalg.norm(parameters[i:i+3])
            i += 3

        if self.calibrate_arm:
            arm_offset_xyz_length = np.linalg.norm(parameters[i:i+3])
            i += 3
            arm_offset_rotvec_mag = np.linalg.norm(parameters[i:i+3])
            i += 3

            
        ###########
        # Compute the individual parameter errors and sum them up.
        
        parameter_error = 0.0

        if self.calibrate_controller_offsets: 
            # no penalty region: +/- 10 deg for the pan and tilt angle
            # offsets (10 deg = math.pi/18.0 rad)
            parameter_error += ca.soft_constraint_error(pan_angle_offset_abs, math.pi/18.0, 10.0)
            parameter_error += ca.soft_constraint_error(tilt_angle_offset_abs, math.pi/18.0, 10.0)

        if self.calibrate_pan_backlash:
            # no penalty region: +/- 5 deg for the pan and tilt angle
            # offsets (5 deg = math.pi/36.0 rad)
            parameter_error += ca.soft_constraint_error(pan_looked_left_offset_abs, math.pi/36.0, 10.0)
            i += 1
            
        if self.calibrate_tilt_backlash:
            # no penalty region: +/- 5 deg for the pan and tilt angle
            # offsets (5 deg = math.pi/36.0 rad)
            parameter_error += ca.soft_constraint_error(tilt_looking_up_offset_abs, math.pi/36.0, 10.0)
            i += 1
            
        if self.calibrate_arm_backlash:
            # no penalty region: +/- 1 cm 
            parameter_error += ca.soft_constraint_error(arm_retracted_offset_abs, 0.01, 10.0)
            i += 1

        # no penalty region: 0.5cm radius sphere within which the head
        # assembly can be placed relative to its uncalibrated position
        # with respect to the mast. 5 rotation of the head (5 deg =
        # 180/36 deg = math.pi/36.0 rad)
        parameter_error += ca.soft_constraint_error(head_assembly_offset_xyz_length, 0.005, 10.0)
        parameter_error += ca.soft_constraint_error(head_assembly_offset_rotvec_mag, math.pi/36.0, 10.0)
            
        # no penalty region: 1cm radius sphere within which the pan
        # assembly can be placed relative to its uncalibrated position
        # with respect to the mast. 5 degree max rotation of the pan
        # assembly (5 deg = 180/36 deg = math.pi/36.0 rad)
        parameter_error += ca.soft_constraint_error(pan_assembly_offset_xyz_length, 0.01, 10.0)
        parameter_error += ca.soft_constraint_error(pan_assembly_offset_rotvec_mag, math.pi/36.0, 10.0)

        # no penalty region: 1cm radius sphere within which the tilt
        # assembly can be placed relative to its uncalibrated position
        # with respect to the pan assembly. 5 degree max rotation of
        # the tilt assembly. (5 deg = 180/36 deg = math.pi/36.0 rad)
        parameter_error += ca.soft_constraint_error(tilt_assembly_offset_xyz_length, 0.01, 10.0)
        parameter_error += ca.soft_constraint_error(tilt_assembly_offset_rotvec_mag, math.pi/36.0, 10.0)
        
        # no penalty region: 0.5cm radius sphere within which the
        # camera can be placed relative to its uncalibrated position
        # with respect to the pan assembly. 5 max rotation of the
        # camera assembly. (5 deg = 180/36 deg = math.pi/36.0 rad)
        parameter_error += ca.soft_constraint_error(camera_mount_offset_xyz_length, 0.005, 10.0)
        parameter_error += ca.soft_constraint_error(camera_mount_offset_rotvec_mag, math.pi/36.0, 10.0)

        if self.calibrate_lift:
            # no penalty region: 1cm radius sphere within which the
            # mast can be placed relative to its uncalibrated position
            # with respect to the mobile base. 1.5 degree max rotation
            # of the mast relative to the mobile base. (2.5 deg =
            # 180/72 deg = math.pi/72.0 rad)
            parameter_error += ca.soft_constraint_error(lift_offset_xyz_length, 0.01, 10.0)
            parameter_error += ca.soft_constraint_error(lift_offset_rotvec_mag, math.pi/72.0, 10.0)
            
        if self.calibrate_arm: 
            # no penalty region: 1cm radius sphere within which the
            # arm mount can be placed relative to its uncalibrated
            # position with respect to the shoulder. 5 degree max
            # rotation of the arm relative to the shoulder carriage (5
            # deg = 180/36 deg = math.pi/36.0 rad).
            parameter_error += ca.soft_constraint_error(arm_offset_xyz_length, 0.01, 10.0)
            parameter_error += ca.soft_constraint_error(arm_offset_rotvec_mag, math.pi/36.0, 10.0)

        output_error = {'total': parameter_error}
        return output_error
    
    
    def update_urdf(self, parameters):
        # Updates self.new_urdf using the provided parameter
        # vector.

        # Unpack the parameter vector into relevant variables.
        i = 0

        if self.calibrate_controller_offsets:
            self.pan_angle_offset = parameters[i]
            i += 1
            self.tilt_angle_offset = parameters[i]
            i += 1
        else:
            self.pan_angle_offset = 0.0
            self.tilt_angle_offset = 0.0
            
        if self.calibrate_pan_backlash:
            self.pan_looked_left_offset = parameters[i]
            i += 1

        if self.calibrate_tilt_backlash:
            self.tilt_looking_up_offset = parameters[i]
            i += 1

        if self.calibrate_arm_backlash:
            self.arm_retracted_offset = parameters[i]
            i += 1

        head_assembly_offset_xyz = parameters[i:i+3]
        i += 3
        head_assembly_offset_rotvec = parameters[i:i+3]
        i += 3
            
        pan_assembly_offset_xyz = parameters[i:i+3]
        i += 3
        pan_assembly_offset_rotvec = parameters[i:i+3]
        i += 3

        tilt_assembly_offset_xyz = parameters[i:i+3]
        i += 3
        tilt_assembly_offset_rotvec = parameters[i:i+3]
        i += 3

        camera_mount_offset_xyz = parameters[i:i+3]
        i += 3
        camera_mount_offset_rotvec = parameters[i:i+3]
        i += 3

        if self.calibrate_lift:
            lift_offset_xyz = parameters[i:i+3]
            i += 3
            lift_offset_rotvec = parameters[i:i+3]
            i += 3

        if self.calibrate_arm:
            arm_offset_xyz = parameters[i:i+3]
            i += 3
            arm_offset_rotvec = parameters[i:i+3]
            i += 3

        # Update the URDF.
        self.camera_pan_joint.origin.xyz = self.original_pan_assembly_xyz + pan_assembly_offset_xyz
        pan_assembly_r = Rotation.from_rotvec(pan_assembly_offset_rotvec) * self.original_pan_assembly_r
        self.camera_pan_joint.origin.rpy = pan_assembly_r.as_euler('xyz')
        
        self.camera_tilt_joint.origin.xyz = self.original_tilt_assembly_xyz + tilt_assembly_offset_xyz
        tilt_assembly_r = Rotation.from_rotvec(tilt_assembly_offset_rotvec) * self.original_tilt_assembly_r
        self.camera_tilt_joint.origin.rpy = tilt_assembly_r.as_euler('xyz')

        self.camera_mount_joint.origin.xyz = self.original_camera_mount_xyz + camera_mount_offset_xyz
        camera_mount_r = Rotation.from_rotvec(camera_mount_offset_rotvec) * self.original_camera_mount_r
        self.camera_mount_joint.origin.rpy = camera_mount_r.as_euler('xyz')

        if self.calibrate_lift: 
            self.lift_joint.origin.xyz = self.original_lift_xyz + lift_offset_xyz
            lift_r = Rotation.from_rotvec(lift_offset_rotvec) * self.original_lift_r
            self.lift_joint.origin.rpy = lift_r.as_euler('xyz')

        if self.calibrate_arm: 
            self.arm_joint.origin.xyz = self.original_arm_xyz + arm_offset_xyz
            arm_r = Rotation.from_rotvec(arm_offset_rotvec) * self.original_arm_r
            self.arm_joint.origin.rpy = arm_r.as_euler('xyz')

            
    def calculate_normalization(self):
        # Finds the number of observations that will be used for each
        # type of error measurement.
        for e in self.error_measures:
            e.reset_observation_count()
        for i, s in enumerate(self.data):
            if self.use_this_sample(i, s): 
                for e in self.error_measures:
                    e.increment_observation_count(s)
        print('Number of observations of each error measurement type to be used from the data:')
        self.observations_used_for_fit = {}
        for e in self.error_measures:
            print(e.name, '=', e.number_of_observations)
            self.observations_used_for_fit[e.name] = e.number_of_observations

    def generate_target_visualizations(self):
        target_markers = []
        marker_id = 333333
        marker_time = self.node.get_clock().now().to_msg()
        # infinite lifetime (I think.)
        #lifetime = rospy.Duration()
        # 10 second lifetime, which assumes that these will be periodically resent.
        duration = Duration(seconds=10.0)
        lifetime = duration.to_msg()
        rgba = [1.0, 1.0, 1.0, 1.0]
        
        for i, s in enumerate(self.data):
            if self.use_this_sample(i, s):

                sample = deepcopy(s)

                # Update the sample to account for controller
                # parameters and backlash. Specifically, this adds pan
                # and tilt offsets and backlash state dependent
                # offsets to pan, tilt, and the arm extension.
                self.update_sample_with_offsets_and_backlash(sample)
                
                for e in self.error_measures:
                    markers, marker_id = e.get_target_ros_markers(sample, marker_id, marker_time, rgba, self.gravity_acceleration_vec)
                    for m in markers:
                        unique = True
                        for current_m in target_markers:
                            same_type = (m.type == current_m.type)
                            same_pose = (m.pose == current_m.pose)
                            same_points = (m.points == current_m.points)
                            if same_type and same_pose and same_points:
                                unique = False
                        if unique:
                            m.lifetime = lifetime
                            target_markers.append(m)
        # Note that there will be many target markers for markers on a
        # kinematic chain sensitive to the joint values, such as the
        # ArUco markers on the wrist. In contrast, the ArUco markers
        # on the mobile base will only have a single unique marker.
        return target_markers
    
                            
    def visualize_fit(self, parameters_to_fit):
        # Visualize the fit of the provided parameter vector. Also
        # calculate the fit error, display it, and display an error if
        # it is too high.
        self.visualize = True
        self.infinite_duration_visualization = True
        fit_error = self.calculate_error(parameters_to_fit)
        self.check_fit_error(fit_error)
        

    def update_sample_with_offsets_and_backlash(self, sample):
        # This changes the sample passed as an argument using
        # controller parameters and the backlash state. Specifically,
        # it adds pan and tilt offsets and backlash state dependent
        # offsets to pan, tilt, and the arm extension.
        
        backlash_state = sample.get('backlash_state', self.default_backlash_state)

        if backlash_state['joint_head_pan_looked_left']:
            pan_backlash_correction = self.pan_looked_left_offset
        else:
            pan_backlash_correction = 0.0
            
        if backlash_state['joint_head_tilt_looking_up']:
            tilt_backlash_correction = self.tilt_looking_up_offset
        else:
            tilt_backlash_correction = 0.0
            
        if backlash_state['wrist_extension_retracted']: 
            arm_backlash_correction = self.arm_retracted_offset
        else: 
            arm_backlash_correction = 0.0

        joints = sample['joints']
        for j in self.telescoping_joints:
            joints[j] = joints[j] + (arm_backlash_correction/4.0)

        joints['joint_head_pan'] = joints['joint_head_pan'] + self.pan_angle_offset + pan_backlash_correction
        joints['joint_head_tilt'] = joints['joint_head_tilt'] + self.tilt_angle_offset + tilt_backlash_correction

        
    def calculate_error(self, parameters_to_fit, output_error_terms=False):
        # Calculates the fit error for the provide parameter
        # vector. This can output the total error by itself or both
        # the total error and the individual unweighted error terms
        # depending on the output_error_terms argument.
        
        # Initialize the error terms. 
        error_terms = self.initial_error_terms.copy()

        # Find the error based solely on the parameter vector values.
        error_terms['parameter_error_total'] += self.parameter_error_term(parameters_to_fit)['total']

        # Updates the URDF and related method variables using the
        # current parameter vector.
        self.update_urdf(parameters_to_fit)

        # Initialize marker array for visualization.
        marker_array = MarkerArray()
        marker_time = self.node.get_clock().now().to_msg()
        marker_id = 0

        if self.visualize:
            # Send the target markers on each iteration, since they
            # can be changed by some of the fit parameters, such as
            # backlash compensation for arm retraction.
            self.target_markers = self.generate_target_visualizations()        
            marker_array.markers.extend(self.target_markers)

        # Iterate through all the observations (samples) in the data.
        for i, s in enumerate(self.data):

            # Only use selected samples.
            if self.use_this_sample(i, s):

                # Do not alter the original data.
                sample = deepcopy(s)
                backlash_state = sample.get('backlash_state', self.default_backlash_state)
        
                # Update the sample to account for controller
                # parameters and backlash. Specifically, this adds pan
                # and tilt offsets and backlash state dependent
                # offsets to pan, tilt, and the arm extension.
                self.update_sample_with_offsets_and_backlash(sample)
                
                # Update each marker, including whether it was
                # detected and the joint values used to compute the
                # URDF's marker prediction in the world frame (not
                # through the camera).
                for e in self.error_measures:
                    e.update(sample, marker_time, self.gravity_acceleration_vec)

                # Find the transform from the camera's coordinate
                # system to the world frame, since all errors are
                # computed with respect to the world frame.
                joints = sample['joints']
                self.camera_transform = self.camera_chain.get_affine_matrix(joints)

                # If the wrist has a large downward deflection due to
                # a heavy tool, this can potentially be useful. It
                # ignores z-axis error for wrist markers when the arm
                # is extended past a defined length. 
                #
                # wrist_extension = reduce(lambda current_sum, joint_name: current_sum + joints[joint_name], self.telescoping_joints, 0.0)
                # max_wrist_extension_for_z_fit = 0.13
                # if wrist_extension > max_wrist_extension_for_z_fit:
                #     fit_z = False
                # else:
                #     fit_z = True
                fit_z = True
                
                # Whether or not to use ArUco marker orientation in
                # the objective function.
                #fit_orientation = False
                fit_orientation = True

                # Iterate through the error measures in order to
                # calculate the errors for this sample and add them to
                # the error terms.
                for e in self.error_measures:
                    if e.detected:
                        # Only use this error measure if the sample
                        # includes the required observation.
                        
                        if e.location == 'wrist':
                            # fit_z is only relevant to the wrist,
                            # which could deflect downward.
                            delta_error, ros_markers, marker_id = e.error(self.camera_transform, fit_z, fit_orientation, marker_id, self.visualize)
                        else:
                            delta_error, ros_markers, marker_id = e.error(self.camera_transform, True, fit_orientation, marker_id, self.visualize)

                        # Typically there will be far fewer
                        # observations made with the head tilted
                        # upwards. This weight can help compensate for
                        # this imbalance.
                        if backlash_state['joint_head_tilt_looking_up']:
                            tilt_error_multiplier = self.error_weights['joint_head_tilt_looking_up_error_multiplier']
                        else:
                            tilt_error_multiplier = 1.0

                        # Add the errors to the relevant error
                        # terms. For example, there can be both a
                        # position and an orientation error.
                        for d in delta_error:
                            error_term_name = e.name + '_' + d
                            error_terms[error_term_name] += (tilt_error_multiplier * delta_error[d])
                            
                        # Collects markers to visualize.
                        marker_array.markers.extend(ros_markers)

        # Calculate the total error by summing the weighted error
        # terms.
        total_error = 0.0
        for k in error_terms:
            total_error += self.error_weights[k] * error_terms[k]
        print('error_terms =', error_terms)
        print('total_error =', total_error)

        if self.visualize:
            if self.infinite_duration_visualization:
                for m in marker_array.markers:
                    duration = Duration()
                    m.lifetime = duration.to_msg()
            self.visualization_markers_pub.publish(marker_array)

        # Returns the total error and the individual, unweighted error
        # terms.
        if output_error_terms:
            return total_error, error_terms

        # Only returns the total error.
        return total_error

    
    def save_urdf_file(self, time_string=None):
        if time_string is None: 
            time_string = ca.create_time_string()
        urdf_string = self.new_urdf.to_xml_string()
        filename = self.calibration_directory + 'head_calibrated_' + time_string + '.urdf'
        print('Saving new URDF file to ', filename)
        fid = open(filename, 'w')
        fid.write(urdf_string)
        fid.close()
        print('Finished saving')

        
    def save_controller_calibration_file(self, time_string=None):
        if time_string is None: 
            time_string = ca.create_time_string()

        controller_parameters = self.get_controller_parameters()
        
        no_numpy_controller_parameters = {}
        for key, entry in controller_parameters.items():
            if "tolist" in dir(entry):
                entry = entry.tolist()
            no_numpy_controller_parameters[key] = entry

        filename = self.calibration_directory + 'controller_calibration_head_' + time_string + '.yaml'
        print('Saving controller calibration head parameters to a YAML file named ', filename)
        fid = open(filename, 'w')
        yaml.dump(no_numpy_controller_parameters, fid)
        fid.close()
        print('Finished saving.')

        

class ProcessHeadCalibrationDataNode(Node):

    def __init__(self, opt_results_file_to_load=None, load_most_recent_opt_results=False, visualize_only=False, visualize=True):
        self.opt_results_file_to_load = opt_results_file_to_load
        self.load_most_recent_opt_results = load_most_recent_opt_results
        self.visualize_only = visualize_only
        self.visualize = visualize
        self.loaded_data = None
        
        # different error tolerances for different speeds and qualities of fit
        self.fit_quality_dict = {'fastest_lowest_quality': 0.1,
                                 'medium_speed_and_quality': 0.01,
                                 'slow_high_quality': 0.001,
                                 'slowest_highest_quality': 0.0001}

        self.data_to_use_dict = {
            'use_very_little_data': ca.use_very_little_data,
            'use_all_aruco_and_some_accel': ca.use_all_aruco_and_some_accel, 
            'use_all_data': ca.use_all_data
        }

            
    def main(self, use_check_calibration_data):
        
        rclpy.init()
        node = rclpy.create_node('process_head_calibration_data')
        self.node_name = node.get_name()        
        node.get_logger().info("{0} started".format(self.node_name))

        # TODO: Check significance of ~
        self.calibration_directory = node.get_parameter('~calibration_directory')
        node.get_logger().info('Using the following directory for calibration files: {0}'.format(self.calibration_directory))

        # load parameters for what data to fit and how well to fit it
        #TODO: Check parameter type
        head_calibration_options = node.get_parameter('/head_calibration_options')
        self.data_to_use = head_calibration_options['data_to_use']
        self.fit_quality = head_calibration_options['fit_quality']
        if self.data_to_use not in self.data_to_use_dict.keys():
            node.get_logger().error('Unrecognized option: data_to_use = {0}, valid options are {1}'.format(self.data_to_use, self.data_to_use_dict.keys())) 
        if self.fit_quality not in self.fit_quality_dict.keys():
            node.get_logger().error('Unrecognized option: fit_quality = {0}, valid options are {1}'.format(self.data_to_use, self.fit_quality_dict.keys())) 
        node.get_logger().info('data_to_use = {0}'.format(self.data_to_use))
        node.get_logger().info('fit_quality = {0}'.format(self.fit_quality))

        self.cma_tolfun = self.fit_quality_dict[self.fit_quality]
        self.sample_selector_func = self.data_to_use_dict[self.data_to_use]
        
        # load default tilt backlash transition angle
        # TODO: Check significance of ~
        self.uncalibrated_controller_calibration_filename = node.get_parameter('~uncalibrated_controller_calibration_filename')
        node.get_logger().info('Loading factory default tilt backlash transition angle from the YAML file named {0}'.format(self.uncalibrated_controller_calibration_filename))
        fid = open(self.uncalibrated_controller_calibration_filename, 'r')
        default_controller_parameters = yaml.load(fid)
        fid.close()
        tilt_angle_backlash_transition_rad = default_controller_parameters['tilt_angle_backlash_transition']
        deg_per_rad = 180.0/math.pi
        node.get_logger().info('self.tilt_angle_backlash_transition_rad in degrees = {0}'.format(tilt_angle_backlash_transition_rad * deg_per_rad))

        # TODO: Check significance of ~
        self.uncalibrated_urdf_filename = node.get_parameter('~uncalibrated_urdf_filename')
        node.get_logger().info('The uncalibrated URDF filename: {0}'.format(self.uncalibrated_urdf_filename))
        
        if self.load_most_recent_opt_results or (self.opt_results_file_to_load is not None): 
            if self.load_most_recent_opt_results: 
                filenames = glob.glob(self.calibration_directory + 'head_calibration_result' + '_*[0-9].yaml')
                filenames.sort()
                filename = filenames[-1]
                print('Loading most recent CMA-ES result from a YAML file named ' + filename)
            else:
                filename = self.opt_results_file_to_load
                print('Loading CMA-ES result from a YAML file named ' + filename)
            fid = open(filename, 'r')
            cma_result = yaml.load(fid)
            fid.close()

            show_data_used_during_optimization = True
            if show_data_used_during_optimization: 
                # attempt to visualize with the same data that was used during the optimization
                self.data_to_use = cma_result.get('data_to_use', 'use_all_data')
            else:
                # show all data
                self.data_to_use = 'use_all_data'
                
            self.sample_selector_func = self.data_to_use_dict[self.data_to_use]
           
            calibration_options = cma_result.get('calibration_options', {})
            fit_parameters = np.array(cma_result['best_parameters'])
            self.calibrator = HeadCalibrator(node, self.uncalibrated_urdf_filename, self.calibration_directory, self.sample_selector_func, calibration_options, self.visualize, tilt_angle_backlash_transition_rad)
            
            if self.visualize_only:
                print('Loading the most recent data file.')
                self.calibrator.load_data(fit_parameters, use_check_calibration_data=use_check_calibration_data)
                print('Visualizing how well the model fits the data.')
                print('Wait to make sure that RViz has time to load.')
                # TODO: Replace time.sleep() instances with rclpy.clock.sleep_for() instances
                time.sleep(5.0)
                self.calibrator.visualize_fit(fit_parameters)
            else: 
                time_string = ca.create_time_string()
                print('Updating the URDF with the parameters previously optimized with CMA-ES.') 
                self.calibrator.update_urdf(fit_parameters)
                print('Saving the updated URDF file.') 
                self.calibrator.save_urdf_file(time_string)
                print('Saving the updated controller calibration file.') 
                self.calibrator.save_controller_calibration_file(time_string)

        else:
            
            start_time = time.time()
        
            calibration_options = {'calibrate_lift': True,
                                   'calibrate_arm': True,
                                   'calibrate_controller_offsets': True,
                                   'calibrate_pan_backlash': True,
                                   'calibrate_tilt_backlash': True,
                                   'calibrate_arm_backlash': True}

            self.calibrator = HeadCalibrator(self.uncalibrated_urdf_filename, self.calibration_directory, self.sample_selector_func, calibration_options, self.visualize, tilt_angle_backlash_transition_rad)
            parameter_names = self.calibrator.get_names_of_parameters_to_fit()
            
            #"incumbent solution"
            all_options = cma.CMAOptions()
            #"termination criterion: tolerance in function value"
            options = {'tolfun': self.cma_tolfun}

            num_parameters = len(parameter_names)
            initial_solution = num_parameters * [0.0]
            self.calibrator.load_data(initial_solution)
            initial_standard_deviation = 0.1
            es = cma.CMAEvolutionStrategy(initial_solution, initial_standard_deviation, options)
            es.optimize(self.calibrator.calculate_error)
            print
            print('Optimization complete.')
            print
            es.result_pretty()

            end_time = time.time()
            calibration_time_in_minutes = (end_time - start_time)/60.0
            print('Minutes spent calibrating: ', calibration_time_in_minutes)

            # "
            # A results tuple from CMAEvolutionStrategy property result.
            # This tuple contains in the given position and as attribute
            #     0 xbest best solution evaluated
            #     1 fbest objective function value of best solution
            #     2 evals_best evaluation count when xbest was evaluated
            #     3 evaluations evaluations overall done
            #     4 iterations
            #     5 xfavorite distribution mean in "phenotype" space, to be considered as current best estimate of the optimum
            #     6 stds effective standard deviations, can be used to compute a lower bound on the expected coordinate-wise distance to the true optimum, which is (very) approximately stds[i] * dimension**0.5 / min(mueff, dimension) / 1.5 / 5 ~ std_i * dimension**0.5 / min(popsize / 2, dimension) / 5, where dimension = CMAEvolutionStrategy.N and mueff = CMAEvolutionStrategy.sp.weights.mueff ~ 0.3 * popsize.
            # "
            # documentation copied from
            # http://cma.gforge.inria.fr/apidocs-pycma/cma.evolution_strategy.CMAEvolutionStrategyResult.html

            # Get best error terms
            best_parameters = es.result[0]
            best_total_error, best_error_terms = self.calibrator.calculate_error(best_parameters, output_error_terms=True)
            for key in best_error_terms:
                best_error_terms[key] = float(best_error_terms[key])
            
            # Convert from Numpy arrays to human-readable lists
            no_numpy_cma_result = []
            for entry in es.result:
                if "tolist" in dir(entry):
                    entry = entry.tolist()
                no_numpy_cma_result.append(entry)

            # Create a dictionary with the parameters to improve human
            # readability
            best_parameter_dict = {}
            for name, value in zip(parameter_names, no_numpy_cma_result[0]):
                best_parameter_dict[name] = value
                
            cma_result = {
                'calibration_time_in_minutes': calibration_time_in_minutes,
                'tilt_angle_backlash_transition': self.calibrator.tilt_angle_backlash_transition_rad,
                'initial_solution': initial_solution,
                'initial_standard_deviation': initial_standard_deviation,
                'options': options,
                'calibration_options': self.calibrator.get_calibration_options(), 
                'parameter_names': parameter_names,
                'best_parameters': no_numpy_cma_result[0],
                'best_parameter_dict': best_parameter_dict,
                'best_parameters_error': no_numpy_cma_result[1],
                'best_parameters_error_terms': best_error_terms,
                'num_evals_to_find_best': no_numpy_cma_result[2],
                'num_evals_total': no_numpy_cma_result[3],
                'cma_iterations': no_numpy_cma_result[4],
                'cma_parameter_means': no_numpy_cma_result[5],
                'cma_parameter_stddevs': no_numpy_cma_result[6],
                'fit_quality_dict': self.fit_quality_dict,
                'fit_quality': self.fit_quality,
                'data_to_use': self.data_to_use,
                'observations_used_for_fit': self.calibrator.observations_used_for_fit,
                'calibration_data_filename': self.calibrator.head_calibration_data_filename,
                'error_weights': self.calibrator.error_weights
            }

            fit_error = cma_result['best_parameters_error']
            self.calibrator.check_fit_error(fit_error)
            
            time_string = ca.create_time_string()

            filename = self.calibration_directory + 'head_calibration_result_' + time_string + '.yaml'
            if not self.visualize_only:
                print()
                print('********************************************************')
                filename = self.calibration_directory + 'head_calibration_result_' + time_string + '.yaml'
                print('Saving CMA-ES result to a YAML file named ', filename)
                fid = open(filename, 'w')
                yaml.dump(cma_result, fid)
                fid.close()
                print('Finished saving.')

                fit_parameters = cma_result['best_parameters']
                self.calibrator.update_urdf(fit_parameters)

                self.calibrator.save_urdf_file(time_string)
                self.calibrator.save_controller_calibration_file(time_string)
            else:
                print()
                print('********************************************************')
                print('Not saving due to visualize only mode')
                print()
                print('Would have saved the CMA-ES result to a YAML file named ', filename)
                print()
                print('The following cma_result would have been saved:')
                print(cma_result)


def main():
    parser = ap.ArgumentParser(description='Process head calibration data and work with resulting files.')
    parser.add_argument('--load', action='store', help='Do not perform an optimization and instead load the specified file, which should contain CMA-ES optimization results.', default=None)
    parser.add_argument('--load_prev', action='store_true', help='Do not perform an optimization and instead load the most recent CMA-ES optimization results.')
    parser.add_argument('--only_vis', action='store_true', help='Only visualize the fit of the CMA-ES optimization results. This does not save any results.')
    parser.add_argument('--no_vis', action='store_true', help='Do not calculate or publish any visualizations. This results in faster fitting.')
    parser.add_argument('--check', action='store_true', help='Use data collected to check the current calibration, instead of data collected to fit a new calibration.')
    
    args, unknown = parser.parse_known_args()

    opt_results_file_to_load = args.load
    load_most_recent_opt_results = args.load_prev
    visualize_only = args.only_vis
    turn_on_visualization = not args.no_vis
    use_check_calibration_data = args.check
    
    node = ProcessHeadCalibrationDataNode(opt_results_file_to_load = opt_results_file_to_load, load_most_recent_opt_results = load_most_recent_opt_results, visualize_only = visualize_only, visualize=turn_on_visualization)
    node.main(use_check_calibration_data)


if __name__ == '__main__':
    main()
