#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32

class testSubscriber():
  def __init__(self):
    self.test_sub = rospy.Subscriber('/test', Float32, self.float_callback)
  
  def float_callback(self, data):
    integer = data.data
    print(integer)
    
if __name__ == "__main__":
  rospy.init_node('test2')
  test_sub = testSubscriber()
  rospy.spin()