#! /usr/bin/env python

import rospy
from gpiozero import DistanceSensor
from std_msgs.msg import Float32

class UltrasonicSensor():
  def __init__(self, max_distance, echo, trigger, sensor_num):
    self.sensor = DistanceSensor(max_distance, echo, trigger)
    self.ultrasonic_pub = rospy.Publisher('ultrasonic_sensor_'+ str(sensor_num), Float32, queue_size=3)
    
    while not rospy.is_shutdown():
      self.ultrasonic_pub.publish(self.sensor.distance * 100)