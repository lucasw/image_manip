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

    images = {}
    images['mosaic'] = 'mosaic.jpg'
    images['gradient_spiral'] = 'gradient_spiral.png'
    list_images = []

    image_manip_dir = get_package_share_directory('image_manip')
    print('image_manip dir ' + image_manip_dir)
    for key in images.keys():
        node_name = key + '_publisher'
        # params = prefix + node_name + '.yaml'
        launches.append(launch_ros.actions.Node(
            package='image_manip', node_executable='image_publisher',
            node_name=node_name, output='screen',
            arguments=[image_manip_dir + '/data/' + images[key]],
            remappings=[('image_raw', key),
            ]))

        image_out = key + '_resized'
        list_images.append(image_out)

        node_name = key + '_resize'
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
                ('image_in', key),
                ('image_out', image_out),
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
                        blue = 68,
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
                        num_b = 3,
                        b0 = 1.0,
                        b1 = 0.5,
                        b2 = -0.5,
                        ))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
                package='image_manip', node_executable='iir_image',
                node_name=node_name, output='screen',
                arguments=['__params:=' + params],
                remappings=[
                    ('image_0', 'color'),
                    ('image_1', list_images[0]),
                    ('image_2', list_images[1]),
                    ('image_out', 'iir_image'),
                ]))

    launches.append(launch_ros.actions.Node(
                package='usb_cam', node_executable='show_image.py',
                node_name=node_name, output='screen',
                arguments=['__params:=' + params],
                remappings=[
                    ('image_raw', 'iir_image'),
                ]))

    return launch.LaunchDescription(launches)
