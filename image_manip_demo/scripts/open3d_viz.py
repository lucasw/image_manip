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
    Image,
)


def depth_to_pcd(depth_np: np.ndarray, camera_info: CameraInfo, roi_x=0, roi_y=0):
    K = camera_info.K
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]

    width = depth_np.shape[1]
    height = depth_np.shape[0]
    roi_cx = cx - roi_x
    roi_cy = cy - roi_y
    o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, roi_cx, roi_cy)
    o3d_image = o3d.geometry.Image(depth_np)
    text = f"focal: {o3d_intrinsic.get_focal_length()}, principal {o3d_intrinsic.get_principal_point()}"
    text += f" {depth_np.shape} {roi_x} {roi_y}"
    rospy.loginfo_throttle(0.0, text)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_image, o3d_intrinsic, depth_scale=1.0)
    return pcd


class Open3DViz:
    def __init__(self):
        self.lock = Lock()

        self.viz = None
        self.old_pcd = None
        self.new_pcd = None

        self.cv_bridge = CvBridge()

        # TODO(lucasw) use dynamic reconfigure or at least rosparams
        self.roi_x = 200  # 1000
        self.roi_w = 300
        self.roi_y = 100
        self.roi_h = 500

        camera_info_sub = message_filters.Subscriber("camera_info", CameraInfo)
        depth_sub = message_filters.Subscriber("depth/image", Image)
        subs = [depth_sub, camera_info_sub]
        # self._ets = message_filters.TimeSynchronizer(subs, queue_size=15)
        self.ets = message_filters.ApproximateTimeSynchronizer(subs, queue_size=15, slop=0.1)
        self.ets.registerCallback(self.callback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.update, reset=True)

    def update(self, event: rospy.timer.TimerEvent):
        if self.viz is None:
            self.viz = o3d.visualization.Visualizer()
            # this only updates once
            # self.viz = o3d.visualization.VisualizerWithEditing()
            self.viz.create_window()
            self.view_control = self.viz.get_view_control()
            # self.view_control.change_field_of_view(10)
            # rospy.loginfo(self.view_control.get_field_of_view())
            self.viz.get_render_option().point_size = 1.5

        with self.lock:
            new_pcd = self.new_pcd
            self.new_pcd = None

        if new_pcd is not None:
            view_param = None
            if self.old_pcd is not None:
                # https://github.com/isl-org/Open3D/issues/4306
                # https://github.com/isl-org/Open3D/issues/2264#issuecomment-1410327023
                view_param = self.view_control.convert_to_pinhole_camera_parameters()

                # don't remove unless there are new pcds to replace these
                for pcd in self.old_pcd:
                    self.viz.remove_geometry(pcd)

            for ind, pcd in enumerate(new_pcd):
                if ind > 0:
                    pcd.paint_uniform_color([0.5, 0.206, 0.8])
                else:
                    pcd.paint_uniform_color([0.1, 0.306, 0.1])
                self.viz.add_geometry(pcd)

            if view_param is not None:
                self.view_control.convert_from_pinhole_camera_parameters(view_param)
            else:
                # the get fov is degrees, but not clear what change_field_of_view units are
                # degrees / 5.0 ??
                rospy.loginfo(self.view_control.get_field_of_view())
                self.view_control.change_field_of_view(-2.0)
                rospy.loginfo(self.view_control.get_field_of_view())

            self.old_pcd = new_pcd

        self.viz.poll_events()
        self.viz.update_renderer()

    def callback(self, depth_image: Image, camera_info: CameraInfo):

        t0 = rospy.Time.now()

        x0 = self.roi_x
        y0 = self.roi_y

        if self.roi_w is None:
            x1 = depth_image.width
        else:
            x1 = x0 + self.roi_w

        if self.roi_h is None:
            y1 = depth_image.height
        else:
            y1 = y0 + self.roi_h

        depth_np = self.cv_bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)

        roi_depth_np = depth_np[y0:y1, x0:x1].copy()
        roi_pcd = depth_to_pcd(roi_depth_np, camera_info, roi_x=x0, roi_y=y0)

        depth_np[y0:y1, x0:x1] = np.nan

        pcd = depth_to_pcd(depth_np, camera_info)

        # rospy.loginfo_throttle(2.0, f"{x0} {x1}, {y0} {y1}")

        t1 = rospy.Time.now()
        rospy.loginfo_throttle(2.0, f"{pcd} {roi_pcd} open3d processing time {(t1 - t0).to_sec()}s")
        # o3d.visualization.draw_geometries([pcd])
        with self.lock:
            self.new_pcd = [pcd, roi_pcd]


if __name__ == "__main__":
    rospy.init_node("open3d_viz")
    node = Open3DViz()
    rospy.spin()
