#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import xacro


def generate_launch_description():
    
    package_name = 'twheel_description'
    rviz_file_name = 'twheel_config.rviz'
    world_path='world/my_world.sdf'
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')
    
    robot_description_path = os.path.join(get_package_share_directory(
                                    'twheel_description'), 
                                    'urdf',
                                    'twheel.urdf')
    
    robot_description = xacro.process_file(robot_description_path).toxml()

    

    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[{'robot_description': robot_description}]
    )
    
    joint_state_publisher = Node(package='joint_state_publisher',
                                    executable='joint_state_publisher',
                                    name='joint_state_publisher'
    )
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                        executable='joint_state_publisher_gui',
                                        name='joint_state_publisher_gui'
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
                '-entity', 'twheel_description',
                '-x', '0',
                '-y', '0',
                '-z', '1',
                '-file', 'twheel.urdf',
        ]
    )


    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    # launch_description.add_action(joint_state_publisher_gui)
    # launch_description.add_action(spawn_entity)
    # launch_description.add_entity

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    