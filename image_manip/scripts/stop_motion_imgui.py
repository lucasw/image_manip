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
        images = ['diff_image', 'blur_image', 'live_image_small', 'anim']
        for image in images:
            req = AddWindow.Request()
            req.name = image + " viewer"
            req.topic = "/" + image
            req.type = AddWindow.Request.IMAGE
            self.future = self.add_window_cli.call_async(req)

        req = AddWindow.Request()
        req.name = "Capture"
        req.topic = "/capture_single"
        req.type = AddWindow.Request.PUB
        req.sub_type = AddWindow.Request.BOOL
        req.value = 0.0
        req.min = 0.0
        req.max = 1.0
        self.future = self.add_window_cli.call_async(req)

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
            else:
                v4l_controls[name][field] = res.values[i].string_value
            # if res.values[i].type == ParameterType.PARAMETER_INTEGER:
            # elif res.values[i].type == ParameterType.PARAMETER_STRING:

        for key in v4l_controls.keys():
            control = v4l_controls[key]
            print(control)
            req = AddWindow.Request()
            req.name = control['name']
            req.topic = control['topic']
            req.type = AddWindow.Request.PUB
            # TODO(lucasw) handle bools and menus
            req.sub_type = AddWindow.Request.INT32
            # TODO(lucasw) need to get current value
            req.value = float(control['default'])
            req.min = float(control['min'])
            req.max = float(control['max'])
            print(req)
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
