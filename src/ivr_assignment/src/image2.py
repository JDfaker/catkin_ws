#!/usr/bin/env python

#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from scipy.optimize import least_squares
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
    self.circles1_p_sub = rospy.Subscriber("/image1/circles_p", Float64MultiArray, self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    #____publish end effector position____#
    self.end_effector_pub = rospy.Publisher("end_effector_position",Float64MultiArray,queue_size = 10)
    #______publish_target_position_____#
    self.target_3Dposition_pub = rospy.Publisher("/target/position_estimation",Float64MultiArray,queue_size = 10)
    #______control_part____#
    self.beginning_time = rospy.get_time()
    self.time_previous_step = np.array([rospy.get_time()],dtype = 'float64')
    self.error = np.array([0.0, 0.0, 0.0], dtype = 'float64')
    self.error_d = np.array([0.0, 0.0, 0.0], dtype = 'float64')
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size = 10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size = 10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size = 10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size = 10)

  def callback1(self,data):
    circles_pos1 = np.array(data.data)
    self.yellow_proj_pos1 =  np.array([circles_pos1[0],circles_pos1[1]])
    self.blue_proj_pos1 = np.array([circles_pos1[2],circles_pos1[3]])
    self.green_proj_pos1 = np.array([circles_pos1[4],circles_pos1[5]])
    self.red_proj_pos1 = np.array([circles_pos1[6],circles_pos1[7]])
    self.target_proj_pos1 = np.array([circles_pos1[8],circles_pos1[9]])


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
    self.circles_3D_position = self.estimate_3Dposition()

    #____publish end effector position____#
    self.end_effector_pos = Float64MultiArray()
    self.end_effector_pos.data = self.circles_3D_position[3]
    self.end_effector_pub.publish(self.end_effector_pos)

    #____target_part_____#
    self.target_3Dposition = self.estimate_target_3Dposition()
    self.target = Float64MultiArray()
    self.target.data = self.target_3Dposition
    self.target_3Dposition_pub.publish(self.target)

    #______contol_part__________#
    q_d = self.control_closed()
    self.joint1 = Float64()
    self.joint2 = Float64()
    self.joint3 = Float64()
    self.joint4 = Float64()
    self.joint1.data = q_d[0]
    self.joint2.data = q_d[1]
    self.joint3.data = q_d[2]
    self.joint4.data = q_d[3]
    self.robot_joint1_pub.publish(self.joint1)
    self.robot_joint2_pub.publish(self.joint2)
    self.robot_joint3_pub.publish(self.joint3)
    self.robot_joint4_pub.publish(self.joint4)

    try:
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

  #estimate all circles 3D position
  def estimate_3Dposition(self):
    a = self.pixel2meter()
    yellow_circle3D_pos = np.array([0, 0, 0])
    blue_circle3D_pos = np.array([0,0,2])
    #if there is no green in image2, assume x = 0, get z from image1
    if(self.green_proj_pos2[0] == -1):
      green_x = 0
      green_y = (self.green_proj_pos1[0] - self.yellow_proj_pos1[0]) * a
      green_z = (self.yellow_proj_pos1[1] - self.green_proj_pos1[1]) * a
    #if there is no green is image1, assume y = 0, get z from image2
    if(self.green_proj_pos1[0] == -1) :
      green_x = (self.green_proj_pos2[0] - self.yellow_proj_pos2[0]) * a
      green_y = 0
      green_z = (self.yellow_proj_pos2[1] - self.green_proj_pos2[1]) * a
    if(self.green_proj_pos1[0] != -1 and self.green_proj_pos2[0] != -1):
      green_x = (self.green_proj_pos2[0] - self.yellow_proj_pos2[0]) * a
      green_y = (self.green_proj_pos1[0] - self.yellow_proj_pos1[0]) * a
      green_z = (self.yellow_proj_pos2[1] - self.green_proj_pos2[1]) * a
    green_circle3D_pos = np.array([green_x,green_y,green_z])

    if(self.red_proj_pos2[0] == -1):
      red_x = green_x
      red_y = (self.red_proj_pos1[0] - self.yellow_proj_pos1[0]) * a
      red_z = (self.yellow_proj_pos1[1] - self.red_proj_pos1[1]) * a
    if(self.red_proj_pos1[0] == -1):
      red_x = (self.red_proj_pos2[0] - self.yellow_proj_pos2[0]) * a
      red_y = green_y
      red_z = (self.yellow_proj_pos2[1] - self.red_proj_pos2[1]) * a
    if(self.red_proj_pos1[0] != -1 and self.red_proj_pos2[0] != -1):
      red_x = (self.red_proj_pos2[0] - self.yellow_proj_pos2[0]) * a
      red_y = (self.red_proj_pos1[0] - self.yellow_proj_pos1[0]) * a
      red_z = (self.yellow_proj_pos2[1] - self.red_proj_pos2[1]) * a
    red_circle3D_pos = np.array([red_x,red_y,red_z])
    return np.array([yellow_circle3D_pos,blue_circle3D_pos,green_circle3D_pos,red_circle3D_pos])

  def pixel2meter(self):
    dist = self.yellow_proj_pos2[1] - self.blue_proj_pos2[1]
    return 2.0/dist

  def detect_green(self,image):
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if(M['m00'] == 0):
      print("Error: can not find the greed")
      return np.array([-1,-1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_red(self,image):
    mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if (M['m00'] == 0):
      print("Error: can not find the red")
      return np.array([-1,-1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_blue(self,image):
    mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if (M['m00'] == 0):
      print("Error: can not find the blue")
      return np.array([-1, -1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_yellow(self,image):
    mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if (M['m00'] == 0):
      print("Error: can not find the yellow")
      return np.array([-1, -1])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  #get a mask of box and the target
  def detect_orange(self,image):
    mask = cv2.inRange(image,(50,100,110),(90,185,220))
    return mask

  def find_target(self,image,template):
    res = cv2.matchTemplate(image,template,1)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    return np.array([min_loc[0],min_loc[1]])

  def estimate_target_3Dposition(self):
    a = self.pixel2meter()
    orange_mask = self.detect_orange(self.cv_image2)
    template = cv2.imread("image_crop.png", 0)
    target_proj_pos2 = self.find_target(orange_mask, template)
    target_x = (target_proj_pos2[0] - self.yellow_proj_pos2[0]) * a - 1
    target_y = (self.target_proj_pos1[0] - self.yellow_proj_pos1[0]) * a
    target_z = (self.yellow_proj_pos1[1] - self.target_proj_pos1[1]) * a - 0.7
    return np.array([target_x,target_y,target_z])

  #____kinematic matrix for estimation of j4______#
  def kinematic_matrix(self, q):
    s1 = math.sin(self.j1)
    s2 = math.sin(self.j2)
    s3 = math.sin(self.j3)
    s4 = math.sin(q[0])
    c1 = math.cos(self.j1)
    c2 = math.cos(self.j2)
    c3 = math.cos(self.j3)
    c4 = math.cos(q[0])
    a3 = 3.5
    a4 = 2.4
    return np.array(
      [a4 * s1 * s2 * c3 * c4 + a4 * c1 * s3 * c4 + a4 * s1 * c2 * s4 + a3 * s1 * s2 * c3 + a3 * c1 * s3 -
       self.circles_3D_position[3][0],
       -a4 * c4 * c1 * s2 * c3 + a4 * c4 * s1 * s3 - a4 * c1 * c2 * s4 - a3 * c1 * s2 * c3 + a3 * s1 * s3 -
       self.circles_3D_position[3][1],
       a4 * c4 * c2 * c3 - a4 * s2 * s4 + a3 * c2 * c3 + 2 - self.circles_3D_position[3][2]])

  def jacobian_matrix(self, q):
    s1 = math.sin(q[0])
    s2 = math.sin(q[1])
    s3 = math.sin(q[2])
    s4 = math.sin(q[3])
    c1 = math.cos(q[0])
    c2 = math.cos(q[1])
    c3 = math.cos(q[2])
    c4 = math.cos(q[3])
    a3 = 3.5
    a4 = 2.4
    return np.array([[a4 * c1 * s2 * c3 * c4 - a4 * s1 * s3 * c4 + a4 * c1 * c2 * s4 + a3 * c1 * s2 * c3 - a3 * s1 * s3,
                      a4 * s1 * c2 * c3 * c4 - a4 * s1 * s2 * s4 + a3 * s1 * c2 * c3,
                      -a4 * s1 * s2 * s3 * c4 + a4 * c1 * c3 * c4 - a3 * s1 * s2 * s3 + a3 * c1 * c3,
                      -a4 * s1 * s2 * c3 * s4 - a4 * c1 * s3 * s4 + a4 * s1 * c2 * c4],
                     [a4 * s1 * s2 * c3 * c4 + a4 * c1 * s3 * c4 + a4 * s1 * c2 * s4 + a3 * s1 * s2 * c3 + a3 * c1 * s3,
                      -a4 * c1 * c2 * c3 * c4 + a4 * c1 * s2 * s4 - a3 * c1 * c2 * c3,
                      a4 * c1 * s2 * s3 * c4 + a4 * s1 * c3 * c4 + a3 * c1 * s2 * s3 + a3 * s1 * c3,
                      a4 * c1 * s2 * c3 * s4 - a4 * s1 * s3 * s4 - a4 * c1 * c2 * c4],
                     [0, -a4 * s2 * c3 * c4 - a4 * c2 * s4 - a3 * s2 * c3, -a4 * c2 * s3 * c4 - a3 * c2 * s3,
                      -a4 * c2 * c3 * s4 - a4 * s2 * c4]])

  #_____kinematic matrix for joint 1 2 3 estimation
  def kinematic03(self, q):
    a3 = 3.5
    return np.array(
      [a3 * math.sin(q[0]) * math.sin(q[1]) * math.cos(q[2]) + a3 * math.cos(q[0]) * math.sin(q[2]) - self.circles_3D_position[2][0],
       -a3 * math.cos(q[0]) * math.sin(q[1]) * math.cos(q[2]) + a3 * math.sin(q[0]) * math.sin(q[2]) -self.circles_3D_position[2][1],
       a3 * math.cos(q[1]) * math.cos(q[2]) + 2 - self.circles_3D_position[2][2]])

  #______jacobian matrix for joint 1 2 3 estimation
  def jacobian03(self, q):
    a3 = 3.5
    return np.array([[a3 * math.cos(q[0]) * math.sin(q[1]) * math.cos(q[2]) - a3 * math.sin(q[0]) * math.sin(q[2]),
                      a3 * math.sin(q[0]) * math.cos(q[1]) * math.cos(q[2]),
                      -a3 * math.sin(q[0]) * math.sin(q[1]) * math.sin(q[2]) + a3 * math.cos(q[0]) * math.cos(q[2])],
                     [a3 * math.sin(q[0]) * math.sin(q[1]) * math.cos(q[2]) + a3 * math.cos(q[0]) * math.sin(q[2]),
                      -a3 * math.cos(q[0]) * math.cos(q[1]) * math.cos(q[2]),
                      a3 * math.cos(q[0]) * math.sin(q[1]) * math.sin(q[2]) + a3 * math.sin(q[0]) * math.cos(q[2])],
                     [0, -a3 * math.sin(q[1]) * math.cos(q[2]), -a3 * math.cos(q[1]) * math.sin(q[2])]])

  def joint_angles_estimation(self):
    res1 = least_squares(self.kinematic03,(0,0,0),self.jacobian03,bounds = (-math.pi / 2, math.pi / 2))
    self.j1 = res1.x[0]
    self.j2 = res1.x[1]
    self.j3 = res1.x[2]
    res2 = least_squares(self.kinematic_matrix,0,bounds = (-math.pi / 2, math.pi / 2))
    self.j4 = res2.x[0]
    return np.array([self.j1,self.j2,self.j3,self.j4])

  def control_closed(self):
    K_p = np.array([[10, 0, 0],[0, 10, 0],[0, 0, 10]]) #P gain
    K_d = np.array([[0.1 ,0, 0],[0, 0.1, 0],[0, 0, 0.1]]) #D gain
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    pos = self.circles_3D_position[3]
    pos_d = self.target_3Dposition
    self.error_d = ((pos_d - pos) - self.error)/dt
    self.error = pos_d - pos

    q = self.joint_angles_estimation() # estimate initial value of joints
    jacobian = self.jacobian_matrix(q)
    J_inv = np.linalg.pinv(jacobian) #psudeo inverse of jacobian
    dq_d = np.dot(J_inv, (np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose())))
    q_d = q + (dt * dq_d)
    return  q_d

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




