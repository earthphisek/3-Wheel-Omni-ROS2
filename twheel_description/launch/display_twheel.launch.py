#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys

#sudo apt install ros-foxy-joint-state-publisher
#sudo apt install ros-foxy-joint-state-publisher-gui
def generate_launch_description():

    # RVIZ
    package_name = 'twheel_description'
    rviz_file_name = 'twheel_config.rviz'
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )
    rviz_Node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')
    robot_desc_path = os.path.join(get_package_share_directory(
                                    'twheel_description'), 
                                    'urdf','visual',
                                    'physical_robot.xacro')

    #robot_description = xacro.process_file(robot_desc_path).toxml()
    robot_description = xacro.process_file(robot_desc_path,mappings={'robot_name' : 'twheel1'}).toxml()
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[{'robot_description': robot_description}]
    )
    # Joint State Publisher & GUI
    joint_state_publisher = Node(package='joint_state_publisher',
                                    executable='joint_state_publisher',
                                    name='joint_state_publisher'
    )
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                        executable='joint_state_publisher_gui',
                                        name='joint_state_publisher_gui'
    )

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(rviz_Node)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    