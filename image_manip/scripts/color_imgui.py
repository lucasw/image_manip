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


class ColorImGui(Node):

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
        req = AddWindow.Request()
        req.name = 'gen color controls'

        # for image in ['image']:
        if True:
            image = 'image'
            widget = Widget()
            widget.name = image + " viewer"
            widget.topic = "/" + image
            widget.type = Widget.IMAGE
            req.widgets.append(widget)

        widget = Widget()
        widget.name = 'frame rate'
        widget.topic = 'gen_color'
        widget.items.append('frame_rate')
        widget.type = Widget.PARAM
        widget.sub_type = Widget.FLOAT32
        widget.value = 1.0
        widget.min = 0.0
        widget.max = 5.0
        req.widgets.append(widget)

        # the imgui color picker would be ideal for this- how to make use of it?
        for channel in ['red', 'green', 'blue']:
            widget = Widget()
            widget.name = channel
            widget.topic = 'gen_color'
            widget.items.append(channel)
            widget.type = Widget.PARAM
            widget.sub_type = Widget.INT32
            widget.value = 128.0
            widget.min = 0.0
            widget.max = 255.0
            req.widgets.append(widget)

        for channel in ['width', 'height']:
            widget = Widget()
            widget.name = channel
            widget.topic = 'gen_color'
            widget.items.append(channel)
            widget.type = Widget.PARAM
            widget.sub_type = Widget.INT32
            widget.value = 128.0
            widget.min = 1.0
            widget.max = 1024.0
            req.widgets.append(widget)

        future = self.add_window_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        print(res)

def main(args=None):
    rclpy.init(args=args)

    demo = ColorImGui()
    demo.run()

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
