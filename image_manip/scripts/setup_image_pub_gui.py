#!/usr/bin/env python3
# Lucas Walter

import argparse
import cv2
import cv_bridge
import math
import time
# import transforms3d
import rclpy
import sys

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3
from imgui_ros.msg import TexturedShape, TfWidget, Widget
from imgui_ros.srv import AddShape, AddTexture, AddTf, AddWindow
from rclpy.node import Node
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import Bool, ColorRGBA, Float32
from tf2_msgs.msg import TFMessage
# pip3 install transforms3d
from transforms3d import _gohlketransforms as tg
from visualization_msgs.msg import Marker

def quat_to_msg(rot):
    quat = Quaternion()
    quat.x = rot[1]
    quat.y = rot[2]
    quat.z = rot[3]
    quat.w = rot[0]
    return quat

class SetupGui(Node):

    def __init__(self, namespace_prefix=""):
        super().__init__('demo')
        self.namespace_prefix = namespace_prefix

        self.bridge = cv_bridge.CvBridge()

        self.cli = self.create_client(AddWindow, 'add_window')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('add_window service not available, waiting again...')

    def wait_for_response(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                if self.future.result() is not None:
                    response = self.future.result()
                    self.get_logger().info(
                        'Result %s' % (str(response)))
                    return response
                else:
                    self.get_logger().info(
                        'Service call failed %r' % (self.future.exception(),))
                break
        return None

    def run(self):
        self.window_name = "roto zoom controls"
        self.add_images()
        # self.add_controls()
        # self.add_tf_widgets()

    def add_tf_widgets(self):
        service_name = 'add_tf'
        self.tf_cli = self.create_client(AddTf, service_name)
        while not self.tf_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('service ' + service_name +
                                   ' not available, waiting again...')
            # TODO(lucasw) time out and give up after a while

        tf_widget = TfWidget()
        tf_widget.name = "opengl to ros"
        tf_widget.window = self.window_name
        tf_widget.min = -1.0
        tf_widget.max = 1.0
        ts = TransformStamped()
        ts.header.frame_id = "map"
        ts.child_frame_id = "floor"
        ts.transform.translation.x = 0.0
        ts.transform.translation.y = 1.0
        ts.transform.translation.z = 0.0
        roll = 0.0
        pitch = -math.pi * 0.5
        yaw = -math.pi * 0.5
        rot = tg.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        ts.transform.rotation = quat_to_msg(rot)
        tf_widget.transform_stamped = ts
        tf_req = AddTf.Request()
        tf_req.tf = tf_widget
        self.future = self.tf_cli.call_async(tf_req)
        self.wait_for_response()

    def add_images(self):
        req = AddWindow.Request()
        req.name = "images"

        image_topics = ['image_raw']
        # image_topics = []

        for topic in image_topics:
            widget = Widget()
            widget.name = topic.replace('/', ' ')
            widget.topic = self.namespace_prefix + '/' + topic
            widget.type = Widget.IMAGE
            req.widgets.append(widget)

        self.future = self.cli.call_async(req)
        self.wait_for_response()

    def add_controls(self):
        req = AddWindow.Request()
        req.name = self.window_name

        node = self.namespace_prefix + '/' + 'roto_zoom'

        widget = Widget()
        widget.name = "frame rate"
        widget.topic = node
        widget.items.append('frame_rate')
        widget.type = Widget.PARAM
        widget.sub_type = Widget.FLOAT32
        widget.value = 0.0
        widget.min = 0.0
        widget.max = 30.0
        req.widgets.append(widget)

        pubs = [['phi', 0.0, -4.0, 4.0],
                ['theta', 0.0, -4.0, 4.0],
                ['psi', 0.0, -4.0, 4.0],
                ['off_x', 0.0, -10.0, 10.0],
                ['off_y', 0.0, -10.0, 10.0],
                ['center_x', 0.0, -10.0, 10.0],
                ['center_y', 0.0, -10.0, 10.0],
                ['center_z', 0.0, -10.0, 10.0],
                ['z', 1.0, 0.01, 5.0],
                ['z_scale', 0.005, 0.0, 0.05],
                ]

        for pub in pubs:
            topic = pub[0]
            val = pub[1]
            min_val = pub[2]
            max_val = pub[3]

            widget = Widget()
            widget.name = topic
            widget.topic = self.namespace_prefix + '/' + topic
            widget.type = Widget.PUB
            widget.sub_type = Widget.FLOAT32
            widget.value = val
            widget.min = min_val
            widget.max = max_val
            req.widgets.append(widget)

        for ctrl in ["width", "height"]:
            widget = Widget()
            widget.name = ctrl
            widget.topic = node
            widget.type = Widget.PARAM
            widget.sub_type = Widget.INT32
            # has to be float even though type above is int
            widget.value = 128.0
            widget.min = 0.0
            widget.max = 2048.0
            req.widgets.append(widget)

        self.future = self.cli.call_async(req)
        self.wait_for_response()

    # TODO(lwalter) will future rclpy Node support now()?
    def tmp_now(self):
        now = time.time()
        stamp = Time()
        stamp.sec = int(now)
        stamp.nanosec = int((now - int(now)) * 1e9)
        return stamp

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='setup gui')
    parser.add_argument('-ns', '--namespace', dest='namespace', type=str,
                        help='namspace to prefix topics with', default='')
    args, unknown = parser.parse_known_args(sys.argv)
    if len(args.namespace) > 0 and args.namespace[-1] == '/':
        args.namespace = args.namespace[:-1]
    print("gui topic namespace " + args.namespace)

    try:
        setup_gui = SetupGui(args.namespace)
        setup_gui.run()
        rclpy.spin(setup_gui)
    finally:
        setup_gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
