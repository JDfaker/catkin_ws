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
    #subscribe the 3d positions of circles from image2
    self.circles1_p_sub = rospy.Subscriber("/image2/circles_p",Float64MultiArray,self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback2)
    self.bridge = CvBridge()
  
  def callback1(self,data):
    circles_xyz = np.array(data.data)
    #self.yellow_pos =  np.array([circles_xyz[0],circles_xyz[1],circles_xyz[2]])
    #self.blue_pos = np.array([circles_xyz[3],circles_xyz[4],circles_xyz[5]])
    #self.green_pos = np.array([circles_xyz[6],circles_xyz[7],circles_xyz[8]])
    self.red_position = np.array([circles_xyz[9],circles_xyz[10],circles_xyz[11]])

  def callback2(self,data):
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

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
    a3 = 3.8571
    a4 = 2.4762
    return np.array(
      [a4*s1*s2*c3*c4 + a4*c1*s3*c4 + a4*s1*c2*s4 + a3*s1*s2*c3 + a3*c1*s3 - self.red_position[0],
      -a4*c4*c1*s2*c3 + a4*c4*s1*s3 - a4*c1*c2*s4 - a3*c1*s2*c3 + a3*s1*s3 - self.red_position[1],
      a4*c4*c2*c3 - a4*s2*s4 + a3*c2*c3 + 2 - self.red_position[2] ])

  def jacobian(self,q):
    s1 = math.sin(q[0])
    s2 = math.sin(q[1])
    s3 = math.sin(q[2])
    s4 = math.sin(q[3])
    c1 = math.cos(q[0])
    c2 = math.cos(q[1])
    c3 = math.cos(q[2])
    c4 = math.cos(q[3])
    a3 = 3.8571
    a4 = 2.4762
    return np.array([[a4*c1*s2*c3*c4 - a4*s1*s3*c4 + a4*c1*c2*s4 + a3*c1*s2*c3 - a3*s1*s3, a4*s1*c2*c3*c4 - a4*s1*s2*s4 + a3*s1*c2*c3, -a4*s1*s2*s3*c4 + a4*c1*c3*c4 - a3*s1*s2*s3 + a3*c1*c3, -a4*s1*s2*c3*s4 - a4*c1*s3*s4 + a4*s1*c2*c4],
      [a4*s1*s2*c3*c4 + a4*c1*s3*c4 + a4*s1*c2*s4 + a3*s1*s2*c3 + a3*c1*s3, -a4*c1*c2*c3*c4 + a4*c1*s2*s4 - a3*c1*c2*c3, a4*c1*s2*s3*c4 + a4*s1*c3*c4 + a3*c1*s2*s3 + a3*s1*c3, a4*c1*s2*c3*s4 - a4*s1*s3*s4 - a4*c1*c2*c4], [0, -a4*s2*c3*c4 - a4*c2*s4 - a3*s2*c3, -a4*c2*s3*c4 - a3*c2*s3, -a4*c2*c3*s4 -a4*s2*c4]])

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


