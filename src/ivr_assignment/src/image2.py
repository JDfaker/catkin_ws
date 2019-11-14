#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    #subscribe the positions of circles from image1
    self.circles1_p_sub = rospy.Subscriber("image1_circles_p",Float64MultiArray,self.callback1)

  def callback1(self,data):
    circles_pos1 = np.array(data.data)
    self.yellow_proj_pos1 =  np.array([circles_pos1[0],circles_pos1[1]])
    self.blue_proj_pos1 = np.array([circles_pos1[2],circles_pos1[3]])
    self.green_proj_pos1 = np.array([circles_pos1[4],circles_pos1[5]])
    self.red_proj_pos1 = np.array([circles_pos1[6],circles_pos1[7]])

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.yellow_proj_pos2 = self.detect_yellow(self.cv_image2)
    self.blue_proj_pos2 = self.detect_blue(self.cv_image2)
    self.green_proj_pos2 = self.detect_green(self.cv_image2)
    self.red_proj_pos2 = self.detect_red(self.cv_image2)

    a = self.estimate_3Dposition()
    print(a)
    # im2=cv2.imshow('window2', self.cv_image2)
    # cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def estimate_3Dposition(self):
    a = self.pixel2meter()
    yellow_circle3D_pos = np.array([0, 0, 0])
    blue_circle3D_pos = np.array([0,0,2])
    green_x = (self.green_proj_pos2[0] - self.yellow_proj_pos2[0]) * a
    green_y = (self.green_proj_pos1[0] - self.yellow_proj_pos1[0]) * a
    green_z = (self.yellow_proj_pos1[1] - self.green_proj_pos1[1]) * a
    green_circle3D_pos = np.array([green_x,green_y,green_z])
    red_x = (self.red_proj_pos2[0] - self.yellow_proj_pos2[0]) * a
    red_y = (self.red_proj_pos1[0] - self.yellow_proj_pos1[0]) * a
    red_z = (self.yellow_proj_pos1[1] - self.red_proj_pos1[1]) * a
    red_circle3D_pos = np.array([red_x,red_y,red_z])
    return np.array([yellow_circle3D_pos,blue_circle3D_pos,green_circle3D_pos,red_circle3D_pos])

  def pixel2meter(self):
    dist = np.sum((self.blue_proj_pos2 - self.yellow_proj_pos2)**2)
    return 2/np.sqrt(dist)

  def detect_green(self,image):
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_red(self,image):
    mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_blue(self,image):
    mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_yellow(self,image):
    mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

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


