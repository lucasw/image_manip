# Copyright 2018 Lucas Walter

import argparse
import launch
import launch_ros.actions
import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory

# TODO(lucasw) put in utility python library file
def write_params(param_file, namespace, node_name, params):
    with open(param_file, 'w') as outfile:
        print("opened '" + param_file + "' for yaml writing '" +
              namespace + "' '" + node_name + "'")
        data = {}
        ns_tokens = []
        tmp_tokens = namespace.strip('/').split('/')
        tmp_tokens.extend(node_name.strip('/').split('/'))
        for tok in tmp_tokens:
            if tok != '':
                ns_tokens.append(tok)
        data[ns_tokens[0]] = {}
        cur_dict = data[ns_tokens[0]]
        for tok in ns_tokens[1:]:
            cur_dict[tok] = {}
            cur_dict = cur_dict[tok]
        cur_dict['ros__parameters'] = params
        yaml.dump(data, outfile, default_flow_style=False)

def get_dir(package_name):
    try:
        package_dir = get_package_share_directory(package_name)
        return package_dir
    except:
        return None
###############################################################################

def generate_launch_description():
    parser = argparse.ArgumentParser(description='image_to_pointcloud')
    parser.add_argument('-ns', '--namespace', dest='namespace', type=str,
                        help='namspace to run nodes in', default='/')
    parser.add_argument('-ig', '--imgui', dest='imgui', action='store_true')
    parser.add_argument('-no-ig', '--no-imgui', dest='imgui', action='store_false')
    parser.set_defaults(imgui=True)
    image_manip_dir = get_dir('image_manip')
    print('image_manip dir ' + image_manip_dir)

    parser.add_argument('-im1', '--image1', dest='image1', type=str,
                        help='first image to publish',
                        default=image_manip_dir + "/data/plasma.png",
                        )
    parser.add_argument('-im2', '--image2', dest='image2', type=str,
                        help='second image to publish',
                        default=image_manip_dir + "/data/mosaic.jpg",
                        )
    args, unknown = parser.parse_known_args(sys.argv)
    print("namespace '" + args.namespace + "'")
    print('use imgui ' + str(args.imgui))

    prefix = '/tmp/ros2/laser_line/'
    if not os.path.exists(prefix):
        os.makedirs(prefix)

    launches = []

    image1 = launch_ros.actions.Node(
            package='image_manip', node_executable='image_publisher', output='screen',
            node_name='image1',
            arguments=[args.image1],
            remappings=[('image_raw', 'image_in')],
            )
    # the background iamge isn't supported yet in roto zoom
    image2 = launch_ros.actions.Node(
            package='image_manip', node_executable='image_publisher', output='screen',
            node_name='image2',
            arguments=[args.image2],
            remappings=[('image_raw', 'background_image')],
            )

    node_name = 'roto_zoom'
    params = dict(
        name = 'roto zoom',
        width = 1000,
        height = 800,
        )
    param_file = prefix + node_name + '.yaml'
    write_params(param_file, args.namespace, node_name, params)
    roto = launch_ros.actions.Node(
            package='image_manip', node_executable='roto_zoom', output='screen',
            node_name=node_name,
            # remappings=[('image_in', 'image1')],
            arguments=['__params:=' + param_file,
                      ],
            )
    launches.extend([image1, image2, roto])

    imgui_ros_dir = get_dir('imgui_ros')
    if args.imgui and imgui_ros_dir is not None:
        node_name = 'imgui_ros'
        params = dict(
            name = 'roto zoom',
            width = 1000,
            height = 800,
            red = 0.2,
            green = 0.2,
            blue = 0.2,
            )
        param_file = prefix + node_name + '.yaml'
        write_params(param_file, args.namespace, node_name, params)
        imgui_ros = launch_ros.actions.Node(
                package='imgui_ros', node_executable='imgui_ros_node', output='screen',
                node_name=node_name,
                arguments=['__params:=' + param_file,
                           # '__ns:=' + args.namespace,
                          ],
                remappings=[],
                )

        setup_gui = launch_ros.actions.Node(
                package='image_manip',
                node_executable='setup_roto_gui.py',
                node_name='setup_gui',
                # we don't want this to run in this namespace, but prefix all the topics with it
                arguments=['-ns=' + args.namespace],
                output='screen',
                )
        launches.extend([imgui_ros, setup_gui])
    else:
        print("couldn't find imgui_ros, launch without it")
        # TODO(lucasw) use rqt_image_view instead to view images

    return launch.LaunchDescription(launches)
