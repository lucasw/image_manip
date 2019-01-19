# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_manip_dir = get_package_share_directory('image_manip')
    print('image_manip dir ' + image_manip_dir)
    image_name = image_manip_dir + "/data/mosaic.jpg"
    image_pub = launch_ros.actions.Node(
            package='image_manip', node_executable='image_publisher', output='screen',
            node_name = 'image_pub',
            parameters = [dict(
                image_name = image_name,
                )],
            )
    resize = launch_ros.actions.Node(
            package='image_manip', node_executable='resize', output='screen',
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            remappings=[('image_in', 'image_raw')])

    node_name = 'imgui_ros'
    params = dict(
        name = 'image resize demo',
        width = 900,
        height = 800,
        )
    imgui_ros = launch_ros.actions.Node(
            package='imgui_ros', node_executable='imgui_ros_node', output='screen',
            node_name=node_name,
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            # arguments=['__params:=' + param_file],
            parameters=[params],
            remappings=[])

    setup_gui = launch_ros.actions.Node(
            package='image_manip', node_executable='resize_imgui.py', output='screen',
            )

    return launch.LaunchDescription([
        image_pub,
        resize,
        imgui_ros,
        setup_gui,
    ])
