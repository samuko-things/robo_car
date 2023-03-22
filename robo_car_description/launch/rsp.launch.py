import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')    
    
     # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robo_car_description'))
    xacro_file = os.path.join(pkg_path,'urdf','urdf_description.xacro')
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()
    
    # robot_description_config= Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])
    # robot_description_xml = ParameterValue(robot_description_config, value_type=str)
  
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_xml, 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )





     # Create the launch description and populate
    ld = LaunchDescription()

    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)

    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher_node)

    
    return ld      # return (i.e send) the launch description for excecution
    
