#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    rospy.init_node("circle_screenshot", anonymous=True)
    self.image = cv2.imread('circle_screenshot.png',0)
    self.image_pub = rospy.Publisher("circle_screenshot",Image, queue_size=10)

    self.bridge = CvBridge()

    self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
    # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
  def timer_cb(self, event):
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image))
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':
    try:
        image = image_converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
