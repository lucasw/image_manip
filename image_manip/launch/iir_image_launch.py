# Copyright 2018 Lucas Walter

import argparse
import launch
import launch_ros.actions
import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    parser = argparse.ArgumentParser(description='usb_cam demo')
    # parser.add_argument('-d', '--device', dest='device', type=str,
    #        help='video device', default='dev/video0')
    parser.add_argument('-wd', '--width', dest='width', type=int,
           help='image width', default=640)
    parser.add_argument('-ht', '--height', dest='height', type=int,
            help='image height', default=480)
    parser.add_argument('-f', '--fps', dest='frame_rate', type=float,
            help='frame rate', default=5)
    args, unknown = parser.parse_known_args(sys.argv[4:])

    launches = []

    width = args.width
    height = args.height
    frame_rate = args.frame_rate

    # write all of the above to various /tmp/ param.yaml files
    # TODO(lucasw) store the parameters in a log location -
    # a directory that is made new every run (with a random string or timestamp in it)
    prefix = "/tmp/ros2/"
    if not os.path.exists(prefix):
        os.makedirs(prefix)

    node_name = 'image_publisher'
    # params = prefix + node_name + '.yaml'
    image_manip_dir = get_package_share_directory('image_manip')
    print('image_manip dir ' + image_manip_dir)
    launches.append(launch_ros.actions.Node(
        package='image_manip', node_executable='image_publisher',
        node_name=node_name, output='screen',
        arguments=[image_manip_dir + '/data/mosaic.jpg'],
        remappings=[('image_raw', 'mosaic'),
        ]))

    node_name = 'resize'
    params = prefix + node_name + '.yaml'
    with open(params, 'w') as outfile:
        print('opened ' + params + ' for yaml writing')
        data = {}
        data[node_name] = dict(ros__parameters = dict(
                    frame_rate = 0.0,
                    width = width,
                    height = height,
                    ))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
        package='image_manip', node_executable='resize',
        node_name=node_name, output='screen',
        arguments=['__params:=' + params],
        remappings=[
            ('image_in', 'mosaic'),
            ('image_out', 'mosaic_resized'),
        ]))

    # generate a gray image for use in image diff
    node_name = 'gray_color'
    params = prefix + node_name + '.yaml'
    with open(params, 'w') as outfile:
        print('opened ' + params + ' for yaml writing')
        data = {}
        data[node_name] = dict(ros__parameters = dict(
                        red = 148,
                        green = 128,
                        blue = 28,
                        width = width,
                        height = height,
                        ))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
                package='image_manip', node_executable='color', output='screen',
                node_name=node_name,
                arguments=['__params:=' + params],
                remappings=[('image', 'color')]))

    # blur the last saved image and the live image
    node_name = 'blur_image'
    params = prefix + node_name + '.yaml'
    with open(params, 'w') as outfile:
        print('opened ' + params + ' for yaml writing')
        data = {}
        data[node_name] = dict(ros__parameters = dict(
                        use_time_sequence = False,
                        num_b = 2,
                        b0 = 0.5,
                        b1 = 0.5,
                        ))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
                package='image_manip', node_executable='iir_image',
                node_name=node_name, output='screen',
                arguments=['__params:=' + params],
                remappings=[
                    ('image_0', 'mosaic_resized'),
                    ('image_1', 'color'),
                    ('image_out', 'blur_image'),
                ]))

    launches.append(launch_ros.actions.Node(
                package='usb_cam', node_executable='show_image.py',
                node_name=node_name, output='screen',
                arguments=['__params:=' + params],
                remappings=[
                    ('image_raw', 'blur_image'),
                ]))

    return launch.LaunchDescription(launches)
