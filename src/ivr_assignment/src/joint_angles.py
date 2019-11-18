#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from scipy.optimize import least_squares
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:


  def __init__(self):
    rospy.init_node('image_processing', anonymous=True)
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.image_sub2 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback2)
    self.bridge = CvBridge()

  def callback2(self,data):
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    green_position = np.array([1,1,2.5])
    res = least_squares(self.fun_Kinematic,green_position,self.jacobian, bounds = ([-np.inf,-np.inf,-np.inf],np.inf))
    print(res.x)
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def fun_Kinematic(self,q):
    return np.array(
      [3*math.sin(q[0])*math.sin(q[1])*math.cos(q[2]) + 3*math.cos(q[0])*math.sin(q[2]),
      -3*math.sin(q[1])*math.cos(q[0])*math.cos(q[2]) + 3*math.sin(q[0])*math.cos(q[2]),
      3*math.cos(q[1])*math.cos(q[2]) + 2])

  def jacobian(self,q):
    return np.array([[3*math.cos(q[0])*math.sin(q[1])*math.cos(q[2]) - 3*math.sin(q[1])*math.cos(q[2]), 3*math.sin(q[0])*math.cos(q[1])*math.cos(q[2]), -3*math.sin(q[0])*math.sin(q[1])*math.sin(q[2]) + 3*math.cos(q[0])*math.cos(q[1])],
                    [3*math.sin(q[0])*math.sin(q[1])*math.cos(q[2]) + 3*math.cos(q[0])*math.cos(q[2]), -3*math.cos(q[0])*math.cos(q[1])*math.cos(q[2]), 3*math.sin(q[1])*math.cos(q[0])*math.cos(q[2]) - 3*math.sin(q[0])*math.sin(q[2])],
                    [0, -3*math.sin(q[1])*math.cos(q[2]), -3*math.sin(q[2])*math.cos(q[1])]])

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


