#!/usr/bin/env python
# Lucas Walter
#
# image utlity functions and classes

import cv2
import struct
import message_filters
import numpy as np
import open3d as o3d
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    Image,
)


# https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
def decode_compressed_depth(msg: CompressedImage):
    # 'msg' as type CompressedImage
    depth_format, compression = msg.format.split(';')
    # remove white space
    depth_format = depth_format.strip()
    compression = compression.strip()
    compression = compression.split(' ')[0]
    if compression != "compressedDepth":
        text = f"Compression type '{compression}' is not 'compressedDepth' '{msg.format}'"
        raise Exception(text)

    # remove header from raw data
    depth_header_size = 12
    raw_header = msg.data[:depth_header_size]
    raw_data = msg.data[depth_header_size:]

    depth_data_raw = cv2.imdecode(np.frombuffer(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
    # rospy.loginfo(f"{depth_data_raw.shape} {depth_data_raw.dtype}")
    if depth_data_raw is None:
        raise Exception("Could not decode compressed depth image,"
                        "probably wrong header size 'depth_header_size'!")

    if depth_format == "16UC1":
        raise Exception(f"Decoding of '{depth_format}' is not implemented!")
    elif depth_format == "32FC1":
        # header: int, float, float
        [compresion_format, depth_quant_a, depth_quant_b] = struct.unpack('iff', raw_header)
        depth_np = depth_quant_a / (depth_data_raw.astype(np.float32) - depth_quant_b)
        # filter max values
        depth_np[depth_data_raw == 0] = 0

        text = f"{depth_quant_a} {depth_quant_b} - {np.max(depth_np)} {np.min(depth_np)}"
        text += f" - {depth_np.shape} {depth_np.dtype}"
        rospy.logdebug(text)
        # depth_np provides distance in meters as f32
        return depth_np
    else:
        raise Exception(f"Decoding of '{depth_format}' is not implemented!")


def camera_to_intrinsic(camera_info: CameraInfo) -> o3d.camera.PinholeCameraIntrinsic:
    K = camera_info.K
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]

    width = camera_info.width
    height = camera_info.height
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return intrinsic


def adjust_intrinsic_roi(intrinsic: o3d.camera.PinholeCameraIntrinsic, width: int, height: int, roi_x=0, roi_y=0):
    (fx, fy) = intrinsic.get_focal_length()
    (cx, cy) = intrinsic.get_principal_point()
    roi_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx - roi_x, cy - roi_y)
    return roi_intrinsic


def depth_color_to_pcd(depth_np: np.ndarray, color_np: np.ndarray, camera_info: CameraInfo,
                       roi_x=0, roi_y=0,
                       depth_scale=1.0, max_depth=10.0):
    height = depth_np.shape[0]
    width = depth_np.shape[1]
    if width == 0 or height == 0:
        rospy.logwarn(f"{depth_np.shape} {color_np.shape}")
        return None
    intrinsic = camera_to_intrinsic(camera_info)
    intrinsic = adjust_intrinsic_roi(intrinsic, width, height, roi_x, roi_y)

    # TODO(lucasw) Image and/or create point cloud function doesn't use array views properly
    if color_np.base is not None:
        # TODO(lucasw) warn the caller?
        color_np = color_np.copy()
    if depth_np.base is not None:
        depth_np = depth_np.copy()

    depth_o3d = o3d.geometry.Image(depth_np)
    color_o3d = o3d.geometry.Image(color_np)
    rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d,
                                                                  convert_rgb_to_intensity=False,
                                                                  depth_scale=depth_scale,
                                                                  depth_trunc=max_depth)
    text = f"focal: {intrinsic.get_focal_length()}, principal {intrinsic.get_principal_point()}"
    text += f" {depth_np.shape} {roi_x} {roi_y}, {rgbd_o3d}"
    rospy.logdebug_throttle(0.0, text)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, intrinsic)  # , depth_scale=1.0)
    return pcd


class DepthColorInfoSub:
    def __init__(self, main_callback, use_compressed_color=True, use_compressed_depth=True,
                 additional_subcribers=[]):
        """
        Example usage in another class, init_node already run:
        use_compressed_color = rospy.get_param("~compressed_color", True)
        use_compressed_depth = rospy.get_param("~compressed_depth", True)
        self.depth_image_info_sub = DepthColorInfoSub(self.callback,
                                                      use_compressed_color,
                                                      use_compressed_depth)

        main_callback takes an uncompressed depth np array, color np array, and camera info message
        """
        self.main_callback = main_callback

        self.cv_bridge = CvBridge()

        camera_info_sub = message_filters.Subscriber("camera_info", CameraInfo)

        if use_compressed_color:
            color_sub = message_filters.Subscriber("image/compressed", CompressedImage)
        else:
            color_sub = message_filters.Subscriber("image", Image)

        if use_compressed_depth:
            depth_sub = message_filters.Subscriber("depth/image/compressedDepth", CompressedImage)
        else:
            depth_sub = message_filters.Subscriber("depth/image", Image)

        if use_compressed_color and use_compressed_depth:
            callback = self.compressed_both_callback
        elif use_compressed_color and not use_compressed_depth:
            callback = self.compressed_color_callback
        elif not use_compressed_color and use_compressed_depth:
            callback = self.compressed_depth_callback
        elif not use_compressed_color and not use_compressed_depth:
            callback = self.uncompressed_callback

        subs = [depth_sub, color_sub, camera_info_sub]
        # self._ets = message_filters.TimeSynchronizer(subs, queue_size=15)
        self.ets = message_filters.ApproximateTimeSynchronizer(subs, queue_size=15, slop=0.1)
        self.ets_used = False
        self.ets.registerCallback(callback)

        rospy.sleep(2.0)
        if not self.ets_used:
            rospy.logwarn("no sync callbacks yet, topics probably aren't in sync or aren't getting published")

    # TODO(lucasw) make these importable also, put whole sync subscriber into re-usable class
    def compressed_both_callback(self, depth_image: Image, compressed_color_image: CompressedImage,
                                 camera_info: CameraInfo):
        depth_np = decode_compressed_depth(depth_image)
        encoding = "bgr8"
        color_np = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_color_image, encoding)
        self.process(depth_np, color_np, camera_info)

    def compressed_color_callback(self, depth_image: Image, compressed_color_image: CompressedImage,
                                  camera_info: CameraInfo):
        depth_np = self.cv_bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)
        encoding = "bgr8"
        color_np = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_color_image, encoding)
        self.process(depth_np, color_np, camera_info)

    def compressed_depth_callback(self, depth_image: CompressedImage, color_image: Image,
                                  camera_info: CameraInfo):
        depth_np = decode_compressed_depth(depth_image)
        encoding = "bgr8"
        color_np = self.cv_bridge.imgmsg_to_cv2(color_image, encoding)
        self.process(depth_np, color_np, camera_info)

    def uncompressed_callback(self, depth_image: Image, color_image: Image,
                 camera_info: CameraInfo):
        depth_np = self.cv_bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)
        encoding = "bgr8"
        color_np = self.cv_bridge.imgmsg_to_cv2(color_image, encoding)
        self.process(depth_np, color_np, camera_info)

    def process(self, depth_np: np.ndarray, color_np: np.ndarray, camera_info: CameraInfo):
        self.ets_used = True
        self.main_callback(depth_np, color_np, camera_info)
