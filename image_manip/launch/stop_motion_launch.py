# Copyright 2018 Lucas Walter

import argparse
import launch
import launch_ros.actions
import os
import sys
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir


def write_params(prefix, ns, node_name, params):
    name = prefix + ns + node_name + '_params.yaml'
    path = os.path.dirname(name)
    if not os.path.exists(path):
        os.makedirs(path)
    with open(name, 'w') as outfile:
        print('opened ' + name + ' for yaml parameter writing')
        data = {}
        data[ns] = {}
        for node_name in params.keys():
            data[ns][node_name] = {}
            data[ns][node_name]['ros__parameters'] = params[node_name]
        yaml.dump(data, outfile, default_flow_style=False)
        return name
    print('error opening file for parameter writing: ' + name)
    return None

def generate_launch_description():
    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-d', '--device', dest='device', type=str,
            help='video device', default='/dev/video0')
    parser.add_argument('-wd', '--width', dest='width', type=int,
            help='image width', default=640)
    parser.add_argument('-ht', '--height', dest='height', type=int,
            help='image height', default=480)
    parser.add_argument('-f', '--fps', dest='frame_rate', type=float,
            help='frame rate', default=5)
    args, unknown = parser.parse_known_args(sys.argv[4:])

    launches = []

    # TODO(lucasw) how to get these from ros2 launch command line?
    device = args.device
    width = args.width
    height = args.height
    frame_rate = args.frame_rate

    decimation = 4
    small_width = int(width / decimation)
    small_height = int(height / decimation)

    launches = []
    composed = False

    cam_params = dict(
            video_device = device,
            framerate = frame_rate,
            io_method = "mmap",
            frame_id = "camera",
            pixel_format = "yuyv",
            image_width = width,
            image_height = height,
            camera_name = "camera",
            )

    deque_params = dict(
            use_time_sequence = False,
            num_b = 2,
            b0 = 0.5,
            b1 = 0.5,
            )

    save_params = dict(
            device = device,
            )

    deque_remappings=[
        ('image', 'live_image_small'),
        # ('captured_image_trigger', 'captured_image_trigger'),
        ]

    if composed:
        # write all of the above to various /tmp/ param.yaml files
        # TODO(lucasw) store the parameters in a log location -
        # a directory that is made new every run (with a random string or timestamp in it)
        prefix = "/tmp/ros2/"
        if not os.path.exists(prefix):
            os.makedirs(prefix)
    else:
        node = launch_ros.actions.Node(
            package='usb_cam',
            node_executable='usb_cam_node',
            node_name='usb_cam',
            output='screen',
            parameters=[cam_params],
            )
        launches.append(node)

        node = launch_ros.actions.Node(
                package='image_manip',
                node_executable='save_image',
                node_name='save_image',
                output='screen',
                parameters=[save_params],
                remappings=[
                    ('image', 'image_raw'),
                    ('single', 'captured_image_trigger'),
                    ],
                )
        launches.append(node)

        # resize the image down for efficiency
        images_in = {}
        images_in['live'] = 'image_raw'
        images_in['saved'] = 'saved_image'
        resize_params = dict(
                frame_rate = 0.0,
                width = small_width,
                height = small_height,
                )
        for key in images_in.keys():
            node = launch_ros.actions.Node(
                package='image_manip',
                node_executable='resize',
                node_name=key + '_resize',
                output='screen',
                parameters=[resize_params],
                remappings=[
                    ('image_in', images_in[key]),
                    ('image_out', key + '_image_small'),
                ])
            launches.append(node)

        node = launch_ros.actions.Node(
            package='image_manip',
            node_executable='image_deque',
            node_name='image_deque',
            output='screen',
            parameters=[deque_params],
            remappings=deque_remappings,
            )
        launches.append(node)

    # camera control
    v4l2ucp_params = dict(
            device = device,
            )
    node = launch_ros.actions.Node(
            package='v4l2ucp',
            node_executable='v4l2ucp_node',
            node_name='v4l2ucp',
            output='screen',
            parameters=[v4l2ucp_params],
            )
    launches.append(node)

    # ros2 topic pub /capture_single std_msgs/Bool "{data: True}" -1

    # blur the last saved image and the live image
    blur_params = dict(
        use_time_sequence = False,
        num_b = 2,
        b0 = 0.5,
        b1 = 0.5,
        )
    node = launch_ros.actions.Node(
        package='image_manip',
        node_executable='iir_image',
        node_name='blur_image',
        output='screen',
        parameters=[blur_params],
        remappings=[
            ('image_0', 'saved_image_small'),
            ('image_1', 'live_image_small'),
            ('image_out', 'blur_image'),
            ],
        )
    launches.append(node)

    # generate a gray image for use in image diff
    gray_params = dict(
            red = 128,
            green = 128,
            blue = 128,
            width = small_width,
            height = small_height,
            frame_rate_ = 1.0,
            )
    node = launch_ros.actions.Node(
            package='image_manip',
            node_executable='color',
            output='screen',
            node_name='gray_color',
            parameters=[gray_params],
            remappings=[('image', 'gray')],
            )
    launches.append(node)

    # diff the gray image with the last saved image and the current live image
    diff_params = dict(
            use_time_sequence = False,
            num_b = 3,
            b0 = 1.0,
            b1 = 0.5,
            b2 = -0.5,
            )
    node = launch_ros.actions.Node(
            package='image_manip',
            node_executable='iir_image',
            node_name='diff_image',
            output='screen',
            parameters=[diff_params],
            remappings=[
                ('image_0', 'gray'),
                ('image_1', 'saved_image_small'),
                ('image_2', 'live_image_small'),
                ('image_out', 'diff_image'),
                ],
            )
    launches.append(node)

    imgui_params = dict(
        name = 'stop motion animation',
        width = 1920,
        height = 990,
        )
    node = launch_ros.actions.Node(
            package='imgui_ros',
            node_executable='imgui_ros_node',
            node_name='imgui_ros',
            output='screen',
            parameters=[imgui_params],
            )
    launches.append(node)

    node = launch_ros.actions.Node(
        package='image_manip',
        node_executable='stop_motion_imgui.py',
        node_name='stop_motion_imgui',
        output='screen',
        )
    launches.append(node)

    return launch.LaunchDescription(launches)
