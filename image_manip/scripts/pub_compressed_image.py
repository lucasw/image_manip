#!/usr/bin/env python
# Copyright 2022 Lucas Walter
# BSD3

# Load a file into a CompressedImage and publish without decoding or looking at the data except to guess
# filetype.
# nominally would be used for jpegs and pngs but any file could go into it

import os

import filetype
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage


class PubCompressedImage:
    def __init__(self):
        self.pub = rospy.Publisher("image/compressed", CompressedImage, queue_size=2)
        filename = os.path.expanduser(rospy.get_param("~image", "image.jpg"))
        rospy.loginfo(filename)

        filetype_guess = filetype.guess(filename)
        encoding = "tbd"
        if filetype_guess is not None:
            encoding = filetype_guess.extension
        rospy.loginfo(encoding)

        frame_id = rospy.get_param("~frame_id", "tbd")

        update_rate = rospy.get_param("~rate", 1.0)
        rate = rospy.Rate(update_rate)

        with open(filename, "rb") as image:
            msg = CompressedImage()
            msg.format = encoding
            msg.header.frame_id = frame_id
            msg.data = np.fromfile(image, np.uint8).tobytes()
            rospy.loginfo(f"loaded {len(msg.data)} bytes")

            while not rospy.is_shutdown():
                msg.header.stamp = rospy.Time.now()
                self.pub.publish(msg)
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pub_compressed_image")
    node = PubCompressedImage()
    rospy.spin()
