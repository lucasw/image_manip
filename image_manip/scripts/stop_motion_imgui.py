#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from imgui_ros.msg import Widget
from imgui_ros.srv import AddWindow
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters

import rclpy
from rclpy.node import Node


class StopMotionImGui(Node):

    def __init__(self):
        super().__init__('stop_motion_imgui')
        self.add_window_cli = self.create_client(AddWindow, 'add_window')
        while not self.add_window_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    # TODO(lucasw) can't this be a callback instead?
    def wait_for_response(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                if self.future.result() is not None:
                    response = self.future.result()
                    self.get_logger().info(
                        'Result %s' % (str(response)))
                else:
                    self.get_logger().info(
                        'Service call failed %r' % (self.future.exception(),))
                break

    def run(self):
        width = (1920 - 400.0) * 0.5
        height = 990.0 * 0.5
        images = ['diff_image', 'blur_image', 'live_image_small', 'anim']
        for i in range(len(images)):
            image = images[i]
            req = AddWindow.Request()
            req.name = image
            req.init = True
            req.position.x = 1920 - width * 2 +  width * (i % 2)
            req.position.y = float(int(i / 2) * height)
            req.size.x = width
            req.size.y = height
            req.scroll_y = 1.0
            widget = Widget()
            widget.name = image + " viewer"
            widget.topic = "/" + image
            widget.type = Widget.IMAGE
            req.widgets.append(widget)
            self.future = self.add_window_cli.call_async(req)

        # Animation controls
        req = AddWindow.Request()
        req.name = 'capture controls'
        req.init = True
        req.position.x = 0.0
        req.position.y = 200.0
        req.size.x = 400.0
        req.size.y = 100.0
        widget = Widget()
        widget.name = "Capture"
        widget.topic = "/capture_single"
        widget.type = Widget.PUB
        widget.sub_type = Widget.BOOL
        widget.value = 0.0
        widget.min = 0.0
        widget.max = 1.0
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "start_index"
        widget.topic = "start_index"
        widget.type = Widget.PUB
        # TODO(lucasw) handle bools and menus
        widget.sub_type = Widget.UINT16
        # TODO(lucasw) need to get current value
        widget.value = float(0)
        widget.min = float(0)
        widget.max = float(256)
        req.widgets.append(widget)

        self.future = self.add_window_cli.call_async(req)

        # Camera controls
        # TODO(lucasw) store this in utility library
        # TODO(lucasw) need to wait for v4l2ucp to be ready,
        # that all the params are ready.
        # But how to read parameters with rclpy
        # ros2 service call /v4l2ucp/list_parameters  rcl_interfaces/ListParameters
        # ros2 service call /v4l2ucp/get_parameters rcl_interfaces/GetParameters
        node = 'v4l2ucp'
        service = node + '/' + 'list_parameters'
        list_cli = self.create_client(ListParameters, service)
        while not list_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(service + ' service not available, waiting again...')
        service = node + '/' + 'get_parameters'
        get_cli = self.create_client(GetParameters, service)
        while not get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(service + ' service not available, waiting again...')

        req = ListParameters.Request()
        future = list_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error("couldn't get parameter list")
            return
        v4l_controls = {}
        names = []
        for full_param in res.result.names:
            param = full_param.split('/')
            if len(param) == 3 and param[0] == 'controls':
                names.append(full_param)
                name = param[1]
                if not name in v4l_controls.keys():
                    v4l_controls[name] = {}
                field = param[2]
                v4l_controls[name][field] = None

        req = GetParameters.Request()
        req.names = names  # [v4l_controls[key]['full_name'] for key in v4l_controls.keys()]
        future = get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error("couldn't get parameter values")
            return
        for i in range(len(res.values)):
            param = names[i].split('/')
            name = param[1]
            field = param[2]
            if field == 'default' or field == 'max'  or field == 'min':
                v4l_controls[name][field] = res.values[i].integer_value
            elif field == 'menu_items':
                v4l_controls[name][field] = res.values[i].string_array_value
            else:
                v4l_controls[name][field] = res.values[i].string_value
            # if res.values[i].type == ParameterType.PARAMETER_INTEGER:
            # elif res.values[i].type == ParameterType.PARAMETER_STRING:

        req = AddWindow.Request()
        req.name = 'camera controls'
        req.init = True
        req.position.x = 0.0
        req.position.y = 300.0
        req.size.x = 400.0
        req.size.y = 690.0
        for key in v4l_controls.keys():
            control = v4l_controls[key]
            print(control)
            widget = Widget()
            widget.name = control['name']
            widget.topic = control['topic']
            widget.type = Widget.PUB
            # TODO(lucasw) handle bools and menus
            if control['type'] == 'menu':
                widget.items = control['menu_items']
                widget.sub_type = Widget.MENU
            else:
                widget.sub_type = Widget.INT32
            # TODO(lucasw) need to get current value
            widget.value = float(control['default'])
            widget.min = float(control['min'])
            widget.max = float(control['max'])
            req.widgets.append(widget)
            # print(widget)

        # control = v4l_controls[key]
        # print(control)

        future = self.add_window_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        print(res)

def main(args=None):
    rclpy.init(args=args)

    demo = StopMotionImGui()
    demo.run()

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
