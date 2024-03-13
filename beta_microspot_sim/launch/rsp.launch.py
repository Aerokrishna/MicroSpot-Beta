import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    urdf_path = os.path.join(get_package_share_directory('beta_microspot_sim'),'description','spot_micro.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_path)

    #rviz file path
    rviz_config_path = os.path.join(get_package_share_directory('beta_microspot_sim'),
                                    'rviz', 'microspot.rviz')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a joint_state_publisher node
    node_joint_state_publisher = Node(
    	package='joint_state_publisher_gui',
    	executable='joint_state_publisher_gui',
    	output='screen'
    )
     #Rviz nodes
    rviz2_node = Node(
         package="rviz2",
         executable="rviz2",
         arguments=['-d', rviz_config_path]
     )
    
    input_node = Node(
         package="beta_microspot",
        #  get_package_share_directory('beta_microspot_sim'),'description','spot_micro.urdf.xacro'
         executable="input_detector.py",
     )
    
    IK_node = Node(
         package="beta_microspot",
         executable="action_3.py",
     )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        node_joint_state_publisher,
        rviz2_node,
        IK_node,
        input_node
    ])
