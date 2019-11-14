#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64


# Publish data
def publish():
  rospy.init_node('publish_node', anonymous=True) #initialize the node
  rate = rospy.Rate(30) # 30hz frequenct (sampling time)
  # initialize a publisher 
  publisher_name = rospy.Publisher("/topic_name", Float64MultiArray, queue_size=10)
  while not rospy.is_shutdown():
    topic_data=Float64MultiArray()
    topic_data.data= [1 , 2]
    publisher_name.publish(topic_data)
    rate.sleep() # makes sure the code is running at the given sampling time



# run the code if the node is called
if __name__ == '__main__':
  try:
    publish()
  except rospy.ROSInterruptException:
    pass


