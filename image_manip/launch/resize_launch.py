# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_manip_dir = get_package_share_directory('image_manip')
    print('image_manip dir ' + image_manip_dir)
    image_pub = launch_ros.actions.Node(
            package='image_manip', node_executable='image_publisher', output='screen',
            arguments=[image_manip_dir + "/data/mosaic.jpg"])
    resize = launch_ros.actions.Node(
            package='image_manip', node_executable='resize', output='screen',
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            remappings=[('image_in', 'image_raw')])
    showimage = launch_ros.actions.Node(
            package='image_tools', node_executable='showimage', output='screen',
            remappings=[('image', 'image_raw')])
    showimage_resized = launch_ros.actions.Node(
            package='image_tools', node_executable='showimage', output='screen',
            remappings=[('image', 'image_out')])
            # arguments=["-t image_raw"])

    return launch.LaunchDescription([
        image_pub,
        showimage,
        resize,
        showimage_resized,
    ])