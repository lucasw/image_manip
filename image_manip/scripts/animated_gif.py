#!/usr/bin/env python
# Load an animated gif and publish the images in sequence (optionally looping)

import cv2
import numpy
import rospy
import sys

from PIL import Image as PILImage
from cv_bridge import CvBridge, CvBridgeError
from scipy import ndimage
from sensor_msgs.msg import Image


rospy.init_node("animated_gif")
gif = rospy.get_param("~gif", "")
rospy.loginfo("loading " + gif)
bridge = CvBridge()
# if gif == "":

# only loads the first frame
# im_array = ndimage.imread(gif)
pil_im = PILImage.open(gif)
im_array = []
try:
    while 1:
        im_array.append(numpy.asarray(pil_im.convert('RGBA')))
        pil_im.seek(pil_im.tell() + 1)
except EOFError:
    pass # end of sequence
print len(im_array)
print im_array[0].shape
pub = rospy.Publisher("image", Image, queue_size=3)

ind = 0
while True:
    for i in range(len(im_array)):
        if rospy.is_shutdown():
            sys.exit(0)
        # print i, type(im_array[i])
        pub.publish(bridge.cv2_to_imgmsg(im_array[i], "rgba8"))
        rospy.sleep(0.05)
