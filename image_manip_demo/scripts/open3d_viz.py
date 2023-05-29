#!/usr/bin/env python
# Lucas Walter
#
# subscribe to a depth image and convert to point cloud in open3d

from threading import Lock

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


def depth_color_to_pcd(depth_np: np.ndarray, color_np: np.ndarray, camera_info: CameraInfo, roi_x=0, roi_y=0):
    height = depth_np.shape[0]
    width = depth_np.shape[1]
    if width == 0 or height == 0:
        return None
    intrinsic = camera_to_intrinsic(camera_info)
    intrinsic = adjust_intrinsic_roi(intrinsic, width, height, roi_x, roi_y)
    depth_o3d = o3d.geometry.Image(depth_np)
    color_o3d = o3d.geometry.Image(color_np)
    rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d,
                                                                  convert_rgb_to_intensity=False)
    text = f"focal: {intrinsic.get_focal_length()}, principal {intrinsic.get_principal_point()}"
    text += f" {depth_np.shape} {roi_x} {roi_y}, {rgbd_o3d}"
    rospy.logdebug_throttle(0.0, text)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, intrinsic)  # , depth_scale=1.0)
    return pcd


class Open3DViz:
    def __init__(self):
        self.lock = Lock()

        self.viz = None
        self.old_geometry = None
        self.new_geometry = None

        self.cv_bridge = CvBridge()

        # TODO(lucasw) use dynamic reconfigure or at least rosparams
        self.roi_x = 200
        self.roi_w = 300
        self.roi_y = 100
        self.roi_h = 500

        use_compressed_image = rospy.get_param("~compressed_image", True)

        camera_info_sub = message_filters.Subscriber("camera_info", CameraInfo)
        # TODO(lucasw) support compressedDepth
        depth_sub = message_filters.Subscriber("depth/image", Image)
        # make this optional
        if use_compressed_image:
            color_sub = message_filters.Subscriber("image/compressed", CompressedImage)
            callback = self.compressed_callback
        else:
            color_sub = message_filters.Subscriber("image", Image)
            callback = self.callback
        subs = [depth_sub, color_sub, camera_info_sub]
        # self._ets = message_filters.TimeSynchronizer(subs, queue_size=15)
        self.ets = message_filters.ApproximateTimeSynchronizer(subs, queue_size=15, slop=0.1)
        self.ets_used = False
        self.ets.registerCallback(callback)

        self.t0 = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update, reset=True)

    def update(self, event: rospy.timer.TimerEvent):
        if not self.ets_used and (event.current_real - self.t0).to_sec() > 2.0:
            rospy.logwarn_once("no sync callbacks yet, topics probably aren't in sync or aren't getting published")
            return

        if self.viz is None:
            self.viz = o3d.visualization.Visualizer()
            # this only updates once
            # self.viz = o3d.visualization.VisualizerWithEditing()
            self.viz.create_window()
            self.view_control = self.viz.get_view_control()
            # self.view_control.change_field_of_view(10)
            # rospy.loginfo(self.view_control.get_field_of_view())
            self.viz.get_render_option().point_size = 1.5
            self.viz.get_render_option().background_color = np.asarray([0.5, 0.5, 0.5])

        with self.lock:
            new_geometry = self.new_geometry
            self.new_geometry = None

        if new_geometry is not None:
            view_param = None
            if self.old_geometry is not None:
                # https://github.com/isl-org/Open3D/issues/4306
                # https://github.com/isl-org/Open3D/issues/2264#issuecomment-1410327023
                view_param = self.view_control.convert_to_pinhole_camera_parameters()

                # don't remove unless there are new pcds to replace these
                for pcd in self.old_geometry:
                    if pcd is None:
                        continue
                    self.viz.remove_geometry(pcd)

            for ind, pcd in enumerate(new_geometry):
                if pcd is None:
                    continue
                # if ind > 0:
                #     pcd.paint_uniform_color([0.5, 0.206, 0.8])
                # else:
                #     pcd.paint_uniform_color([0.1, 0.306, 0.1])
                # TODO(lucasw) draw boudning boxes around each point cloud
                self.viz.add_geometry(pcd)

            if view_param is not None:
                self.view_control.convert_from_pinhole_camera_parameters(view_param)
            else:
                # the get fov is degrees, but not clear what change_field_of_view units are
                # degrees / 5.0 ??
                rospy.loginfo(self.view_control.get_field_of_view())
                self.view_control.change_field_of_view(-2.0)
                rospy.loginfo(self.view_control.get_field_of_view())

            self.old_geometry = new_geometry

        self.viz.poll_events()
        self.viz.update_renderer()

    def compressed_callback(self, depth_image: Image, compressed_color_image: CompressedImage,
                            camera_info: CameraInfo):
        encoding = "bgr8"
        color_np = self.cv_bridge.compressed_imgmsg_to_cv2(compressed_color_image, encoding)
        self.process(depth_image, color_np, camera_info)

    def callback(self, depth_image: Image, color_image: Image, camera_info: CameraInfo):
        color_np = self.cv_bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)
        self.process(depth_image, color_np, camera_info)

    def process(self, depth_image: Image, color_np: np.ndarray, camera_info: CameraInfo):
        self.ets_used = True
        t0 = rospy.Time.now()
        depth_np = self.cv_bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)

        new_geometry = []

        width = depth_image.width
        height = depth_image.height

        x0 = self.roi_x
        y0 = self.roi_y

        if self.roi_w is None:
            x1 = width
        else:
            x1 = x0 + self.roi_w

        if self.roi_h is None:
            y1 = depth_image.height
        else:
            y1 = y0 + self.roi_h

        roi_depth_np = depth_np[y0:y1, x0:x1].copy()
        roi_color_np = color_np[y0:y1, x0:x1, :].copy()
        # rospy.loginfo(f"{depth_np.shape}, {color_np.shape}, {roi_depth_np.shape} {roi_color_np.shape}")
        roi_pcd = depth_color_to_pcd(roi_depth_np, roi_color_np, camera_info, roi_x=x0, roi_y=y0)
        if roi_pcd is not None:
            new_geometry.append(roi_pcd)
            new_geometry.append(roi_pcd.get_axis_aligned_bounding_box())

        depth_np[y0:y1, x0:x1] = np.nan

        pcd = depth_color_to_pcd(depth_np, color_np, camera_info)
        if pcd is not None:
            new_geometry.append(pcd)

        intrinsic = camera_to_intrinsic(camera_info)
        extrinsic = np.zeros((4, 4))
        camera_lines = o3d.geometry.LineSet.create_camera_visualization(intrinsic, extrinsic)
        new_geometry.append(camera_lines)

        # rospy.loginfo_throttle(2.0, f"{x0} {x1}, {y0} {y1}")

        t1 = rospy.Time.now()
        rospy.loginfo_throttle(4.0, f"{pcd} {roi_pcd} open3d processing time {(t1 - t0).to_sec()}s")
        # o3d.visualization.draw_geometries([pcd])
        with self.lock:
            self.new_geometry = new_geometry


if __name__ == "__main__":
    rospy.init_node("open3d_viz")
    node = Open3DViz()
    rospy.spin()
