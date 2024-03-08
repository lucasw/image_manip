#!/usr/bin/env python
# Copyright 2020 Lucas Walter
# Lucas Walter
# Octoboer 2020
# Load a directory of images and publish the images in sequence in a loop
# rosrun image_manip image_folder_publisher.py _folder:=`pwd`

import argparse
import glob
import json
import sys
import traceback
from io import BytesIO

import camera_info_manager
import cv2
import rospy
import zenoh
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo


class ImageFolderPublisher:
    def __init__(self, session: zenoh.session.Session, key: str):
        self.folder = rospy.get_param("~folder", ".")
        self.encoding = rospy.get_param("~encoding", "bgr8")
        self.period = rospy.get_param("~update_period", 0.15)
        rospy.loginfo('update period {}'.format(self.period))
        self.frame = rospy.get_param("~frame", "")

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
        self.image_pub = session.declare_publisher(key)
        # self.image_pub = rospy.Publisher("image", Image, queue_size=3)
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
            # img_msg.header.stamp = event.current_real
            # test
            img_msg.header.stamp.secs = 1
            img_msg.header.stamp.nsecs = 2
            img_msg.header.frame_id = self.frame

            # self.image_pub.publish(img_msg)
            buff = BytesIO()
            img_msg.serialize(buff)
            rospy.loginfo_throttle(1.0, f"put {buff.getbuffer().nbytes} bytes on {self.image_pub}")
            # print([int(v) for v in buff.getvalue()])
            self.image_pub.put(buff.getvalue())

            if self.camera_info_pub is not None:
                camera_info = self.camera_info_manager.getCameraInfo()
                camera_info.header = img_msg.header
                self.camera_info_pub.publish(camera_info)

        except Exception as ex:
            rospy.loginfo('{} {}'.format(str(ex), traceback.format_exc()))
            # raise(ex)


if __name__ == '__main__':
    rospy.init_node("zenoh_image_folder_publisher")
    # remove _ and := args only for rospy init_node
    sys.argv = rospy.myargv()
    sys.argv = [x for x in sys.argv if not x.startswith("_")]

    parser = argparse.ArgumentParser(
        prog='z_pub',
        description='zenoh pub example')
    parser.add_argument('--mode', '-m', dest='mode',
                        choices=['peer', 'client'],
                        type=str,
                        help='The zenoh session mode.')
    parser.add_argument('--connect', '-e', dest='connect',
                        metavar='ENDPOINT',
                        action='append',
                        type=str,
                        help='Endpoints to connect to.')
    parser.add_argument('--listen', '-l', dest='listen',
                        metavar='ENDPOINT',
                        action='append',
                        type=str,
                        help='Endpoints to listen on.')
    # TODO(lucasw) use ros remapping
    parser.add_argument('--key', '-k', dest='key',
                        default='image',
                        type=str,
                        help='The key expression to publish onto.')
    parser.add_argument('--config', '-c', dest='config',
                        metavar='FILE',
                        type=str,
                        help='A configuration file.')

    args = parser.parse_args()
    conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
    if args.mode is not None:
        conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
    if args.connect is not None:
        conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
    if args.listen is not None:
        conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))

    # TODO(lucasw) can I print log message to this?
    # initiate logging
    zenoh.init_logger()

    rospy.loginfo(f"Opening zenoh session with config {conf}")
    session = zenoh.open(conf)
    z_info = session.info()
    rospy.loginfo(f"peers: {z_info.peers_zid()}, routers: {z_info.routers_zid()} {z_info.session} {z_info.zid()}")

    node = ImageFolderPublisher(session, key=args.key)
    rospy.spin()

    del node
    session.close()
