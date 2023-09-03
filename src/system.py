#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray
from distance_sensor_cluster import DistanceSensorCluster
from distance_sensor_info import DistanceSensorInfo
from motor_control import MotorControl
from obstacle_detect import obstacle_detect

class System():
  def __init__(self):
    pin_left_motor_PWM1 = 23
    pin_left_motor_PWM2 = 24
    pin_left_motor_EN = 12
    pin_left_motor_encoder_A = 18
    pin_left_motor_encoder_B = 25
    left_motor_speed_control_kp = 2
    left_motor_speed_control_ki = 1
    left_motor_name = 'left_motor'
    self.left_motor_control = MotorControl(pin_left_motor_PWM1, pin_left_motor_PWM2, pin_left_motor_EN, pin_left_motor_encoder_A, pin_left_motor_encoder_B, left_motor_speed_control_kp, left_motor_speed_control_ki, left_motor_name)
    self.left_motor_control.enable_motor()
    
    pin_right_motor_PWM1 = 4
    pin_right_motor_PWM2 = 17
    pin_right_motor_EN = 27
    pin_right_motor_encoder_A = 5
    pin_right_motor_encoder_B = 6
    right_motor_speed_control_kp = 2
    right_motor_speed_control_ki = 1
    right_motor_name = 'right_motor'
    self.right_motor_control = MotorControl(pin_right_motor_PWM1, pin_right_motor_PWM2, pin_right_motor_EN, pin_right_motor_encoder_A, pin_right_motor_encoder_B, right_motor_speed_control_kp, right_motor_speed_control_ki, right_motor_name)
    self.right_motor_control.enable_motor()
    
    self.front_left_sensor = rospy.Subscriber('/distance_sensor_0', Float32, self.front_left_sensor_cb)
    self.front_right_sensor = rospy.Subscriber('/distance_sensor_1', Float32, self.front_right_sensor_cb)
    self.left_sensor = rospy.Subscriber('/distance_sensor_2', Float32, self.left_sensor_cb)
    self.right_sensor = rospy.Subscriber('/distance_sensor_3', Float32, self.right_sensor_cb)
    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_cb)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_cb)
    self.th = rospy.Subscriber('/state', Float32MultiArray, self.state_cb)
    
    self.front_left_sensor_dist = 200 
    self.front_right_sensor_dist = 200
    self.left_sensor_dist = 200
    self.right_sensor_dist = 200
    self.x = 0
    self.y = 0
    self.th = 0
    
    self.angle_threshold = 0.0785
    
    self.turning_pub = rospy.Publisher('turning', Bool, queue_size=1)
    
  def front_left_sensor_cb(self, data):
    self.front_left_sensor_dist = data.data
    
  def front_right_sensor_cb(self, data):
    self.front_right_sensor_dist = data.data
    
  def left_sensor_cb(self, data):
    self.left_sensor_dist = data.data
    
  def right_sensor_cb(self, data):
    self.right_sensor_dist = data.data
    
  def left_motor_cb(self, data):
    self.left_motor_speed = data.data
    
  def right_motor_cb(self, data):
    self.right_motor_speed = data.data
    
  def state_cb(self, data):
    self.x = data.data[0]
    self.y = data.data[1]
    self.th = data.data[2]

  def drive(self):
    self.left_motor_control.set_motor_speed(0.3)
    self.right_motor_control.set_motor_speed(-0.3)
    rospy.sleep(4.5)
    self.left_motor_control.set_motor_speed(0)
    self.right_motor_control.set_motor_speed(0)
    
  def turn(self, goal_angle):
    self.turning_pub.publish(True)
    Kp = 0.3
    diff = abs(goal_angle - self.th)
    print(diff)
    if goal_angle > self.th and diff > self.angle_threshold:
      self.left_motor_control.set_motor_speed(-Kp * diff)
      self.right_motor_control.set_motor_speed(Kp * diff)
    elif goal_angle < self.th and diff > self.threshold:
      self.left_motor_control.set_motor_speed(Kp * diff)
      self.right_motor_control.set_motor_speed-(Kp * diff)
    else:
      self.left_motor_control.set_motor_speed(0)
      self.right_motor_control.set_motor_speed(0)
      self.turning_pub.publish(False)
      self.angle_threshold = 100
      rospy.sleep(5)
    
if __name__ == "__main__":
  rospy.init_node('system')
  robot = System()
  rospy.sleep(1)
  # robot.drive()
  # rospy.spin()
  while not rospy.is_shutdown():
    try:
      robot.turn(np.pi/2)
    except rospy.ROSInterruptException:
      break