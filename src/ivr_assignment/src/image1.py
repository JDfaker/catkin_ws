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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.image1_circles_pub = rospy.Publisher("/image1/circles_p",Float64MultiArray,queue_size=10)


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #position of target(orange sphere)
    mask = self.detect_orange(self.cv_image1)
    template = cv2.imread("image_crop.png", 0)
    target_p = self.find_target(mask, template)

    self.circles_p = Float64MultiArray()
    yellow_p = self.detect_yellow(self.cv_image1)
    blue_p = self.detect_blue(self.cv_image1)
    green_p = self.detect_green(self.cv_image1)
    red_p = self.detect_red(self.cv_image1)
    self.circles_p.data = np.array([yellow_p[0],yellow_p[1],blue_p[0],blue_p[1],green_p[0],green_p[1],red_p[0],red_p[1],target_p[0],target_p[1]])

    # im1=cv2.imshow('window1', self.cv_image1)
    # cv2.waitKey(1)
    # Publish the results
    try:
      self.image1_circles_pub.publish(self.circles_p)
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_green(self,image):
    mask = cv2.inRange(image,(0,100,0),(0,255,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    M = cv2.moments(mask)
    if (M['m00'] == 0):
      print("Error: can not find the greed")
      return np.array([-1, -1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_red(self,image):
    mask = cv2.inRange(image,(0,0,100),(0,0,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    M = cv2.moments(mask)
    if (M['m00'] == 0):
      print("Error: can not find the red")
      return np.array([-1, -1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_blue(self,image):
    mask = cv2.inRange(image,(100,0,0),(255,0,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    M = cv2.moments(mask)
    if (M['m00'] == 0):
      print("Error: can not find the blue")
      return np.array([-1, -1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_yellow(self,image):
    mask = cv2.inRange(image,(0,100,100),(0,255,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    M = cv2.moments(mask)
    if (M['m00'] == 0):
      print("Error: can not find the yellow")
      return np.array([-1, -1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_orange(self, image):
    mask = cv2.inRange(image, (50, 100, 110), (90, 185, 220))
    return mask

  def find_target(self, image, template):
    res = cv2.matchTemplate(image, template, 0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    return np.array([min_loc[0], min_loc[1]])

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


