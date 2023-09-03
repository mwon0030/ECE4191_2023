#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32

def publish_msg():
  msg_pub = rospy.Publisher('test', Float32, queue_size=1)
  rospy.init_node('test_publisher')
  rate = rospy.Rate(10)
  integer = 0
  while not rospy.is_shutdown():
    msg_pub.publish(integer)
    integer += 1
    rate.sleep()
    
if __name__ == "__main__":
  try:
    publish_msg()
  except rospy.ROSInterruptException:
    pass