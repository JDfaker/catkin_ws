#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64


class subscribe:

  # Define subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('subscriber', anonymous=True)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.subscriber_name = rospy.Subscriber("/topic_name",Float64MultiArray,self.callback)

  # Recieve data for the subscriber
  def callback(self,data):
     print("I heard %s", data.data)
     print(rospy.get_time())


# run the code if the node is called
if __name__ == '__main__':
  subscribe()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 
