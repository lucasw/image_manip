#!/usr/bin/env python
# Lucas Walter
# scale camera info fx/fy from incoming camera info and republish

import copy
from threading import Lock

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from sensor_msgs.msg import CameraInfo


class ScaleCameraInfo:
    def __init__(self):
        self.pub = rospy.Publisher("scaled/camera_info", CameraInfo, queue_size=4)

        self.lock = Lock()
        self.new_config = False
        self.config = None
        ddr = DDynamicReconfigure("")

        ddr.add_variable("enable", "scale or pass through", True)
        ddr.add_variable("scale_fxy", "scale fx and fy", 1.0, 0.05, 10.0)
        ddr.add_variable("scale_dist_k", "scale distortion k", 1.0, 0.0, 20.0)
        ddr.add_variable("scale_dist_p", "scale distortion t", 1.0, 0.0, 20.0)

        self.ddr = ddr
        self.ddr.start(self.config_callback)

        self.sub = rospy.Subscriber("unscaled/camera_info", CameraInfo, self.ci_callback, queue_size=4)

    def config_callback(self, config, event):
        with self.lock:
            # TODO(lucasw) could make self.config private, and set it back to None
            # after getting the new config
            self.config = copy.deepcopy(config)
        return config

    def ci_callback(self, msg):
        with self.lock:
            config = copy.deepcopy(self.config)

        if config.enable:
            # rospy.loginfo(msg.K)
            k = list(msg.K)
            k[0] *= config.scale_fxy
            k[4] *= config.scale_fxy
            msg.K = k

            p = list(msg.P)
            p[0] *= config.scale_fxy
            p[5] *= config.scale_fxy
            msg.P = p

            d = list(msg.D)
            # TODO(lucasw) add per-element scale factors to these
            for i in range(len(d)):
                # TODO(lucasw) not sure about indices, maybe different depending on rational_polynomial vs. plumb_bob
                if i in [2, 3]:
                    d[i] *= config.scale_dist_p
                else:
                    d[i] *= config.scale_dist_k
            msg.D = d

        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("scale_camera_info")
    node = ScaleCameraInfo()
    rospy.spin()
