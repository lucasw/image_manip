#!/usr/bin/env python
# Copyright 2020 Lucas Walter
# Lucas Walter
# Octoboer 2020
# Load a directory of images and publish the images in sequence in a loop
# rosrun image_manip image_folder_publisher.py _folder:=`pwd`

import camera_info_manager
import cv2
import glob
import numpy
import rospy
import sys
import traceback

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image


class ImageFolderPublisher(object):
    def __init__(self):
        self.folder = rospy.get_param("~folder", "")
        self.encoding = rospy.get_param("~encoding", "bgr8")
        self.period = rospy.get_param("~update_period", 0.15)
        rospy.loginfo('update period {}'.format(self.period))
        self.frame = rospy.get_param("~frame", "image_folder")

        self.camera_info_pub = None
        if True:
            self.camera_info_manager = camera_info_manager.CameraInfoManager("tbd_camera_name")
            self.camera_info_url = rospy.get_param("~camera_info_url", "")
            if self.camera_info_url != "":
                rv = self.camera_info_manager.setURL(self.camera_info_url)
                if not rv:
                    rospy.logerr("bad url " + self.camera_info_url)
                else:
                    self.camera_info_manager.loadCameraInfo()
                    self.camera_info_pub = rospy.Publisher("camera_info",
                                                           CameraInfo,
                                                           queue_size=1)

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image", Image, queue_size=3)
        self.image_list = []
        self.image_ind = 1
        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def update(self, event):
        try:
            # TODO(lucasw) make full dir reload optional
            if self.image_ind >= len(self.image_list):
                self.image_ind = 0
                self.image_list = glob.glob(self.folder + "/*.png")
                self.image_list += glob.glob(self.folder + "/*.jpg")
                self.image_list.sort()
                if len(self.image_list) == 0:
                    rospy.logwarn("no images in {}".format(self.folder))
                    return

            name = self.image_list[self.image_ind]
            self.image_ind += 1
            rospy.logdebug(name)
            if self.encoding == "mono8":
                cv_image = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
            else:
                cv_image = cv2.imread(name, cv2.IMREAD_COLOR)

            img_msg = self.bridge.cv2_to_imgmsg(cv_image, self.encoding)
            img_msg.header.stamp = event.current_real
            img_msg.header.frame_id = self.frame
            self.image_pub.publish(img_msg)
            if self.camera_info_pub is not None:
                camera_info = self.camera_info_manager.getCameraInfo()
                camera_info.header = img_msg.header
                self.camera_info_pub.publish(camera_info)

        except Exception as ex:
            rospy.loginfo('{} {}'.format(str(ex), traceback.format_exc()))
            # raise(ex)

if __name__ == '__main__':
    rospy.init_node("image_folder_publisher")
    image_folder_publisher = ImageFolderPublisher()
    rospy.spin()
