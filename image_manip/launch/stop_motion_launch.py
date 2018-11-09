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
    parser.add_argument('-d', '--device', dest='device', type=str,
            help='video device', default='dev/video0')
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

    # write all of the above to various /tmp/ param.yaml files
    # TODO(lucasw) store the parameters in a log location -
    # a directory that is made new every run (with a random string or timestamp in it)
    prefix = "/tmp/ros2/"
    if not os.path.exists(prefix):
        os.makedirs(prefix)

    # usb camera
    usb_cam_params = prefix + "usb_cam.yaml"
    with open(usb_cam_params, 'w') as outfile:
        print("opened " + usb_cam_params + " for yaml writing")
        data = dict(
            usb_cam = dict(
                ros__parameters = dict(
                    video_device = device,
                    framerate = frame_rate,
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

    # camera control
    params = prefix + "v4l2ucp.yaml"
    with open(params, 'w') as outfile:
        print("opened " + params + " for yaml writing")
        data = dict(v4l2ucp = dict(ros__parameters = dict(
                        device = device,
                        )))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
                package='v4l2ucp', node_executable='v4l2ucp_node', output='screen',
                arguments=["__params:=" + params]
                # remappings=[('image_raw', 'image_raw')]
                ))

    params = prefix + "save_image.yaml"
    with open(params, 'w') as outfile:
        print("opened " + params + " for yaml writing")
        data = dict(save_image = dict(ros__parameters = dict(
                device = device,
                )))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
                package='image_manip', node_executable='save_image', output='screen',
                arguments=["__params:=" + params],
                remappings=[
                    ('image', 'image_raw'),
                    ('single', 'captured_image_trigger'),
                ]))

    # resize the image down for efficiency
    images_in = {}
    images_in['live'] = 'image_raw'
    images_in['saved'] = 'saved_image'
    for key in images_in.keys():
        # the name isn't the name of the node, just the executable
        node_name = key + '_resize'
        params = prefix + node_name + '.yaml'
        with open(params, 'w') as outfile:
            print('opened ' + params + ' for yaml writing')
            data = {}
            data[node_name] = dict(ros__parameters = dict(
                        frame_rate = 0.0,
                        width = small_width,
                        height = small_height,
                        ))
            yaml.dump(data, outfile, default_flow_style=False)
        launches.append(launch_ros.actions.Node(
            package='image_manip', node_executable='resize',
            node_name=node_name, output='screen',
            arguments=['__params:=' + params],
            remappings=[
                ('image_in', images_in[key]),
                ('image_out', key + '_image_small'),
            ]))

    # TODO(lucasw) need the image_deque in here to store the animation

    # blur the last saved image and the live image
    params = prefix + "blur_image.yaml"
    node_name = 'blur_image'
    with open(params, 'w') as outfile:
        print("opened " + params + " for yaml writing")
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
                arguments=["__params:=" + params],
                remappings=[('image_0', 'saved_image_small'),
                ('image_1', 'live_image_small'),
                ('image_out', 'blur_image'),
                ]))

    # generate a gray image for use in image diff
    params = prefix + "color.yaml"
    node_name = 'gray_color'
    with open(params, 'w') as outfile:
        print("opened " + params + " for yaml writing")
        data = {}
        data[node_name] = dict(ros__parameters = dict(
                        red = 128,
                        green = 128,
                        blue = 128,
                        ))
        yaml.dump(data, outfile, default_flow_style=False)
    launches.append(launch_ros.actions.Node(
                package='image_manip', node_executable='color', output='screen',
                node_name=node_name,
                arguments=["__params:=" + params],
                remappings=[('image', 'gray')]))

    # diff the gray image with the last saved image and the current live image
    params = prefix + "diff_image.yaml"
    node_name = 'diff_image'
    with open(params, 'w') as outfile:
        print("opened " + params + " for yaml writing")
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
                arguments=["__params:=" + params],
                remappings=[
                    ('image_0', 'gray'),
                    ('image_1', 'saved_image_small'),
                    ('image_2', 'live_image_small'),
                    ('image_out', 'diff_image'),
                    ]))

    # TODO(lucasw)
    if False:
        imgui_ros_dir = get_package_share_directory('imgui_ros')
        if imgui_ros_dir is None:
            pass
            # TODO(lucasw) warning message or use showimage, or rqt
        else:
            pass
            # TODO(lucasw) load up imgui_ros instance and launch a python
            # script that will populate the ui.
    if False:
      image_manip_params = prefix + "image_manip.yaml"
      image_manip_dir = get_package_share_directory('image_manip')
      print('image_manip dir ' + image_manip_dir)
      launches.append(launch_ros.actions.Node(
              package='image_manip', node_executable='image_publisher', output='screen',
              arguments=[image_manip_dir + "/data/mosaic.jpg"]))

    return launch.LaunchDescription(launches)
