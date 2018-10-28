# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_manip_dir = get_package_share_directory('image_manip2')
    print('image_manip2 dir ' + image_manip_dir)
    launches = []
    launches.append(launch_ros.actions.Node(
            package='image_manip2', node_executable='image_publisher', output='screen',
            arguments=[image_manip_dir + "/data/mosaic.jpg"]))
    launches.append(launch_ros.actions.Node(
            package='image_manip2', node_executable='roto_zoom', output='screen',
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            remappings=[('image_in', 'image_raw')]))
    # TODO(lucasw) get this from argument
    show_image = False
    if show_image:
        launches.append(launch_ros.actions.Node(
                package='image_tools', node_executable='showimage', output='screen',
                remappings=[('image', 'image_raw')]))
        launches.append(launch_ros.actions.Node(
                package='image_tools', node_executable='showimage', output='screen',
                remappings=[('image', 'image_out')]))
                # arguments=["-t image_raw"])

    return launch.LaunchDescription(launches)
