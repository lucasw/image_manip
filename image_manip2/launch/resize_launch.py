# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_manip_dir = get_package_share_directory('image_manip2')
    print('image_manip2 dir ' + image_manip_dir)
    image_pub = launch_ros.actions.Node(
            package='image_manip2', node_executable='image_publisher', output='screen',
            arguments=[image_manip_dir + "/data/mosaic.jpg"])
    showimage = launch_ros.actions.Node(
            package='image_tools', node_executable='showimage', output='screen',
            remappings=[('image', 'image_raw')])
            # arguments=["-t image_raw"])

    return launch.LaunchDescription([
        image_pub,
        showimage,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=client,
        #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #     )),
    ])
