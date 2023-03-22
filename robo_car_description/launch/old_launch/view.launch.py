import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro



def generate_launch_description():


    # # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    # # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robo_car_description'))
    xacro_file = os.path.join(pkg_path,'urdf','urdf_description.xacro')
    robot_description_config = xacro.process_file(xacro_file)



    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
        # parameters=[params]
    )

    rviz_config_file_path = os.path.join(pkg_path,'config','robo_car_description_config.rviz')
    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(rviz_config_file_path)]]
        )



    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use sim time if true'),

        node_robot_state_publisher,
        node_joint_state_publisher_gui ,
        rviz2_node,
    ])
