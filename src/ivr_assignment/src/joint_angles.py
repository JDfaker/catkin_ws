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

    self.red_position = np.array([0,2,5])
    res = least_squares(self.fun_Kinematic, (0,0,0,0), self.jacobian, bounds = (-math.pi/2, math.pi/2))
    print 'angles: [%.4f, %.4f, %.4f, %.4f]' %(res.x[0], res.x[1], res.x[2], res.x[3])
    err = self.fun_Kinematic(res.x)
    print 'xyz error: [%.4f, %.4f, %.4f]' %(err[0], err[1], err[2])
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def fun_Kinematic(self,q):
    s1 = math.sin(q[0])
    s2 = math.sin(q[1])
    s3 = math.sin(q[2])
    s4 = math.sin(q[3])
    c1 = math.cos(q[0])
    c2 = math.cos(q[1])
    c3 = math.cos(q[2])
    c4 = math.cos(q[3])
    return np.array(
      [2*s1*s2*c3*c4 + 2*c1*s3*c4 + 2*s1*c2*s4 + 3*s1*s2*c3 + 3*c1*s3 - self.red_position[0],
      -2*c4*c1*s2*c3 + 2*c4*s1*s3 - 2*c1*c2*s4 - 3*c1*s2*c3 + 3*s1*s3 - self.red_position[1],
      2*c4*c2*c3 - 2*s2*s4 + 3*c2*c3 + 2 - self.red_position[2] ])

  def jacobian(self,q):
    s1 = math.sin(q[0])
    s2 = math.sin(q[1])
    s3 = math.sin(q[2])
    s4 = math.sin(q[3])
    c1 = math.cos(q[0])
    c2 = math.cos(q[1])
    c3 = math.cos(q[2])
    c4 = math.cos(q[3])
    return np.array([[2*c1*s2*c3*c4 - 2*s1*s3*c4 + 2*c1*c2*s4 + 3*c1*s2*c3 - 3*s1*s3, 2*s1*c2*c3*c4 - 2*s1*s2*s4 + 3*s1*c2*c3, -2*s1*s2*s3*c4 + 2*c1*c3*c4 - 3*s1*s2*s3 + 3*c1*c3, -2*s1*s2*c3*s4 - 2*c1*s3*s4 + 2*s1*c2*c4],
      [2*s1*s2*c3*c4 + 2*c1*s3*c4 + 2*s1*c2*s4 + 3*s1*s2*c3 + 3*c1*s3, -2*c1*c2*c3*c4 + 2*c1*s2*s4 - 3*c1*c2*c3, 2*c1*s2*s3*c4 + 2*s1*c3*c4 + 3*c1*s2*s3 + 3*s1*c3, 2*c1*s2*c3*s4 - 2*s1*s3*s4 - 2*c1*c2*c4], [0, -2*s2*c3*c4 - 2*c2*s4 - 3*s2*c3, -2*c2*s3*c4 - 3*c2*s3, -2*c2*c3*s4 -2*s2*c4]])

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


