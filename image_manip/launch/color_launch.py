# Copyright 2018 Lucas Walter

import argparse
import launch
import launch_ros.actions
import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory

# TODO(lucasw) namespace ns
def make_node(package, node_executable, node_name, prefix, params, remappings):
    param_file = prefix + node_name + '.yaml'
    with open(param_file, 'w') as outfile:
        print("opened " + param_file + " for yaml writing")
        data = {}
        data[node_name] = dict(ros__parameters = params)
        yaml.dump(data, outfile, default_flow_style=False)
    # TODO(lucasw) check above for failure and return None
    return launch_ros.actions.Node(
                package=package,
                node_executable=node_executable,
                node_name=node_name,
                output='screen',
                arguments=["__params:=" + param_file],
                remappings=remappings
                )

def generate_launch_description():
    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-wd', '--width', dest='width', type=int,
            help='image width', default=640)
    parser.add_argument('-ht', '--height', dest='height', type=int,
            help='image height', default=480)
    parser.add_argument('-r', '--red', dest='red', type=int,
            help='image red', default=128)
    parser.add_argument('-g', '--green', dest='green', type=int,
            help='image green', default=128)
    parser.add_argument('-b', '--blue', dest='blue', type=int,
            help='image blue', default=128)
    parser.add_argument('-f', '--fps', dest='frame_rate', type=float,
            help='frame rate', default=5)
    args, unknown = parser.parse_known_args(sys.argv[4:])

    launches = []

    # write all of the above to various /tmp/ param.yaml files
    # TODO(lucasw) store the parameters in a log location -
    # a directory that is made new every run (with a random string or timestamp in it)
    prefix = "/tmp/ros2/"
    if not os.path.exists(prefix):
        os.makedirs(prefix)

    # generate a gray image for use in image diff
    params = prefix + "color.yaml"
    node_name = 'gen_color'
    with open(params, 'w') as outfile:
        print("opened " + params + " for yaml writing")
        data = {}
        data[node_name] = dict(ros__parameters = dict(
                        red = args.red,
                        green = args.green,
                        blue = args.blue,
                        width = args.width,
                        height = args.height,
                        frame_rate = args.frame_rate,
                        ))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
                package='image_manip', node_executable='color', output='screen',
                node_name=node_name,
                arguments=["__params:=" + params],
                remappings=[('image', 'image')]))

    # TODO(lucasw)
    if True:
        imgui_ros_dir = get_package_share_directory('imgui_ros')
        if imgui_ros_dir is None:
            print('could not find imgui, using showimage instead')
            # TODO(lucasw) the aguments aren't working, but remap does work
            # run image_tools showimage -t /diff_image
            images = ['image']
            for image in images:
                launches.append(launch_ros.actions.Node(
                      package='image_tools', node_executable='showimage', # output='screen',
                      node_name='show_' + image,
                      # arguments=['-t ' + image],
                      remappings=[
                            ('image', image),
                            ]))
        else:
            # TODO(lucasw) load up imgui_ros instance and launch a python
            # script that will populate the ui.
            params = dict(
                    # use_time_sequence = False,
                    # num_b = 2,
                    )
            remappings=[
                # ('image', 'live_image_small'),
                # ('captured_image_trigger', 'captured_image_trigger'),
                ]

            launches.append(make_node(
                package='imgui_ros',
                node_executable='imgui_ros_node',
                node_name='imgui_ros',
                prefix=prefix,
                params=params,
                remappings=remappings))

            launches.append(launch_ros.actions.Node(
                package='image_manip',
                node_executable='color_imgui.py',
                node_name='color_imgui',
                output='screen'))

    if False:
      image_manip_params = prefix + "image_manip.yaml"
      image_manip_dir = get_package_share_directory('image_manip')
      print('image_manip dir ' + image_manip_dir)
      launches.append(launch_ros.actions.Node(
              package='image_manip', node_executable='image_publisher', output='screen',
              arguments=[image_manip_dir + "/data/mosaic.jpg"]))

    return launch.LaunchDescription(launches)
