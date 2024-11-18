from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    paused = DeclareLaunchArgument('paused', default_value='true')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui = DeclareLaunchArgument('gui', default_value='true')
    headless = DeclareLaunchArgument('headless', default_value='false')
    debug = DeclareLaunchArgument('debug', default_value='false')
    rviz = DeclareLaunchArgument('rviz', default_value='false')
    gpu_lidar = DeclareLaunchArgument('gpu_lidar', default_value='false')
    visualize_lidar = DeclareLaunchArgument('visualize_lidar', default_value='false')
    world = DeclareLaunchArgument('world', default_value='worlds/empty.world')
    dex_wrist = DeclareLaunchArgument('dex_wrist', default_value='false')
    publish_upright_img = DeclareLaunchArgument('publish_upright_img', default_value='false',
                                                description='whether to pub rotated upright color image')
    urdf = os.path.join(
        os.path.join(get_package_share_directory('stretch_gazebo'), 'urdf', 'stretch_main.urdf')
    )

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world_name': LaunchConfiguration('world'),
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'verbose': 'true'
        }.items(),
    )

    return LaunchDescription([
        paused,
        use_sim_time,
        gui,
        headless,
        debug,
        rviz,
        gpu_lidar,
        visualize_lidar,
        world,
        dex_wrist,
        publish_upright_img,

        gazebo_launch,

        # Spawning the robot model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            arguments=[
                '-entity', 'stretch', 
                '-topic', 'robot_description'
            ],
            output='screen'
        ),
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )

        # # Robot state publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{'publish_frequency': 30.0}]
        # ),

        # # RViz visualization
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=['-d', FindPackage('stretch_gazebo') + '/config/sim.rviz'],
        #     condition=IfCondition(LaunchConfiguration('rviz'))
        # ),

        # # Loading ROS parameters
        # Node(
        #     package='rosparam',
        #     executable='rosparam',
        #     arguments=['load', FindPackage('stretch_gazebo') + '/config/joints.yaml'],
        #     namespace='stretch_joint_state_controller'
        # ),
        # # Load other ROS parameters similarly...

        # # Controller spawner nodes
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     name='stretch_controller_spawner',
        #     arguments=[
        #         'stretch_joint_state_controller', 'stretch_diff_drive_controller',
        #         'stretch_arm_controller', 'stretch_head_controller',
        #         'stretch_gripper_controller'
        #     ],
        #     condition=UnlessCondition(LaunchConfiguration('dex_wrist'))
        # ),

        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     name='stretch_controller_spawner',
        #     arguments=[
        #         'stretch_joint_state_controller', 'stretch_diff_drive_controller',
        #         'stretch_arm_controller', 'stretch_head_controller',
        #         'stretch_gripper_controller', 'stretch_dex_wrist_controller'
        #     ],
        #     condition=IfCondition(LaunchConfiguration('dex_wrist'))
        # ),

        # # Publishing ground truth odometry
        # Node(
        #     package='stretch_gazebo',
        #     executable='publish_ground_truth_odom.py',
        #     name='publish_ground_truth_odom',
        #     output='screen'
        # ),

        # # UPRIGHT ROTATED CAMERA VIEW
        # Node(
        #     package='image_rotate',
        #     executable='image_rotate',
        #     name='upright_rotater',
        #     condition=IfCondition(LaunchConfiguration('publish_upright_img')),
        #     remappings=[
        #         ('image', '/camera/color/image_raw'),
        #         ('rotated/image', '/camera/color/upright_image_raw')
        #     ],
        #     parameters=[
        #         {'target_frame_id': ''},
        #         {'target_x': -1.5708}
        #     ]
        # )
    ])
