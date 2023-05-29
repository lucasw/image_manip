#!/usr/bin/env python
# Lucas Walter
#
# subscribe to a depth image and convert to point cloud in open3d

from threading import Lock

import message_filters
import open3d as o3d
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import (
    CameraInfo,
    Image,
)


class Open3DViz:
    def __init__(self):
        self.lock = Lock()

        self.viz = None
        self.old_pcd = None
        self.new_pcd = None

        self.cv_bridge = CvBridge()

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
            self.viz.create_window()

        with self.lock:
            new_pcd = self.new_pcd
            self.new_pcd = None

        if new_pcd is not None:
            if self.old_pcd is not None:
                self.viz.remove_geometry(self.old_pcd)

            self.viz.add_geometry(new_pcd)
            self.old_pcd = new_pcd

        self.viz.poll_events()
        self.viz.update_renderer()

    def callback(self, depth_image: Image, camera_info: CameraInfo):
        depth_np = self.cv_bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)

        t0 = rospy.Time.now()

        K = camera_info.K
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]

        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic()
        o3d_intrinsic.set_intrinsics(camera_info.width, camera_info.height, fx, fy, cx, cy)
        o3d_image = o3d.geometry.Image(depth_np)
        pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_image,
                                                              o3d_intrinsic,
                                                              depth_scale=1.0)

        t1 = rospy.Time.now()
        rospy.loginfo(f"open3d processing time {(t1 - t0).to_sec()}s")
        # o3d.visualization.draw_geometries([pcd])
        with self.lock:
            self.new_pcd = pcd


if __name__ == "__main__":
    rospy.init_node("open3d_viz")
    node = Open3DViz()
    rospy.spin()
