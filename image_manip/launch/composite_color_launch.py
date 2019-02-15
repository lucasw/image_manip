# Lucas Walter

import argparse
import launch
import launch_ros.actions
import os
import sys
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir

# this is needed for node composition currently
def write_params(prefix, node_name, params):
    name = prefix + node_name + "_params.yaml"
    path = os.path.dirname(name)
    if not os.path.exists(path):
        os.makedirs(path)
    with open(name, 'w') as outfile:
        print('opened ' + name + ' for yaml parameter writing')
        data = {}
        for ns in params.keys():
            data[ns] = {}
            for node_name in params[ns].keys():
                data[ns][node_name] = {}
                data[ns][node_name]['ros__parameters'] = params[ns][node_name]
        yaml.dump(data, outfile, default_flow_style=False)
        return name
    print('error opening file for parameter writing: ' + name)
    return None

def generate_launch_description():

    prefix = "/tmp/ros2/" + str(int(time.time())) + "/"
    # print('writing launch parameter files to ' + prefix)

    launches = []

    params = {}
    imgui_params = dict(
                name = 'composite color nodes',
                width = 1700,
                height = 900,
                )
    ns_params = {}
    ns_params['imgui_ros'] = imgui_params

    for node_name in ['color_0', 'color_1']:
        image_name = 'image_' + node_name
        # TODO(lucasw) how to do remapping in a composite node for duplicate nodes,
        # want same named topics to be different from each other?
        # For now make the topics determined by parameters
        color_params = dict(
                image = image_name,
                )
        ns_params[node_name] = color_params
        controls_gui = launch_ros.actions.Node(
                package='image_manip',
                node_executable='color_imgui.py',
                node_name=node_name + '_setup_gui',
                arguments=['--node_name', node_name,
                           '--image_name', image_name,
                          ],
                output='screen',
                )
        launches.append(controls_gui)

    composite_node_executable = 'color_imgui_composite'
    params['/'] = ns_params
    param_file = write_params(prefix, composite_node_executable, params)
    params_arg = '__params:=' + param_file

    if True:
        node = launch_ros.actions.Node(
                package='image_manip',
                node_executable=composite_node_executable,
                # can't use node name here, because it will assign it to all the nodes,
                # there will be three different nodes all with the same name.
                # node_name=node_name,
                arguments=[params_arg],
                # parameters=[params],
                # node_namespace=['/'],
                output='screen')
        launches.append(node)

    return launch.LaunchDescription(launches)
