#!/usr/bin/env python
# Lucas Walter
#
# subscribe to a depth image and convert to point cloud in open3d

from threading import Lock

import numpy as np
import open3d as o3d
import rospy
from cv_bridge import CvBridge
from image_manip import (
    DepthColorInfoSub,
    camera_to_intrinsic,
    depth_color_to_pcd,
)
from sensor_msgs.msg import (
    CameraInfo,
)


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

        use_compressed_color = rospy.get_param("~compressed_color", True)
        use_compressed_depth = rospy.get_param("~compressed_depth", True)
        self.depth_image_info_sub = DepthColorInfoSub(self.callback,
                                                      use_compressed_color,
                                                      use_compressed_depth)

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

    def callback(self, depth_np: np.ndarray, color_np: np.ndarray, camera_info: CameraInfo):
        self.ets_used = True
        t0 = rospy.Time.now()
        new_geometry = []

        width = depth_np.shape[1]
        height = depth_np.shape[0]

        x0 = self.roi_x
        y0 = self.roi_y

        if self.roi_w is None:
            x1 = width
        else:
            x1 = x0 + self.roi_w

        if self.roi_h is None:
            y1 = height
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
