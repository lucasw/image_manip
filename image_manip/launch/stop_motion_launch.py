# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions
import os
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launches = []

    # TODO(lucasw) how to get these from ros2 launch command line?
    device = "/dev/video1"
    width = 1920
    height = 1080
    framerate = 5

    small_width = 480
    small_height = 270

    # write all of the above to various /tmp/ param.yaml files
    # TODO(lucasw) store the parameters in a log location -
    # a directory that is made new every run (with a random string or timestamp in it)
    prefix = "/tmp/ros2/"
    if not os.path.exists(prefix):
        os.makedirs(prefix)
    usb_cam_params = prefix + "usb_cam.yaml"
    with open(usb_cam_params, 'w') as outfile:
        print("opened " + usb_cam_params + " for yaml writing")
        data = dict(
            usb_cam = dict(
                ros__parameters = dict(
                    video_device = device,
                    framerate = framerate,
                    io_method = "mmap",
                    frame_id = "camera",
                    pixel_format = "yuyv",
                    image_width = width,
                    image_height = height,
                    camera_name = "camera",
                )
            )
        )
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
        package='usb_cam', node_executable='usb_cam_node', output='screen',
        arguments=["__params:=" + usb_cam_params]
        ))

    v4l2ucp_params = prefix + "v4l2ucp.yaml"
    with open(v4l2ucp_params, 'w') as outfile:
        print("opened " + v4l2ucp_params + " for yaml writing")
        data = dict(v4l2ucp = dict(ros__parameters = dict(
                        device = device,
                        )))
        yaml.dump(data, outfile, default_flow_style=False)

    launches.append(launch_ros.actions.Node(
                package='v4l2ucp', node_executable='v4l2ucp_node', output='screen',
                arguments=["__params:=" + v4l2ucp_params]
                ))

    if False:
      image_manip_params = prefix + "image_manip.yaml"
      image_manip_dir = get_package_share_directory('image_manip')
      print('image_manip dir ' + image_manip_dir)
      launches.append(launch_ros.actions.Node(
              package='image_manip', node_executable='image_publisher', output='screen',
              arguments=[image_manip_dir + "/data/mosaic.jpg"]))
      launches.append(launch_ros.actions.Node(
              package='image_manip', node_executable='resize', output='screen',
              arguments=["__params:=" + image_manip_params],
              remappings=[('image_in', 'image_raw')]))

    return launch.LaunchDescription(launches)
