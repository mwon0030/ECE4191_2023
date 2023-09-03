#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from distance_sensor_cluster import DistanceSensorCluster
from distance_sensor_info import DistanceSensorInfo
from motor_control import MotorControl
from obstacle_detect import obstacle_detect

class System():
  def __init__(self):
    
    # self.front_dist_sensor = DistanceSensorInfo(name = 'front', ECHO = 21, TRIGGER = 20)
    # self.right_dist_sensor = DistanceSensorInfo(name = 'right', ECHO = 26, TRIGGER = 20)
    # self.left_dist_sensor = DistanceSensorInfo(name = 'left', ECHO = 16, TRIGGER = 20)

    # self.distance_sensor_obj_dict = {'front': self.front_dist_sensor, 'right': self.right_dist_sensor, 'left': self.left_dist_sensor}

    # self.dist_sensor_cls = DistanceSensorCluster(self.distance_sensor_obj_dict, 20)
    
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
    
    self.front_left_sensor = rospy.Subscriber('/distance_sensor_0', Float32, self.front_left_sensor_callback)
    self.front_right_sensor = rospy.Subscriber('/distance_sensor_1', Float32, self.front_right_sensor_callback)
    self.left_sensor = rospy.Subscriber('/distance_sensor_2', Float32, self.left_sensor_callback)
    self.right_sensor = rospy.Subscriber('/distance_sensor_3', Float32, self.right_sensor_callback)
    self.left_motor = rospy.Subscriber('/left_motor', Float32, self.left_motor_callback)
    self.right_motor = rospy.Subscriber('/right_motor', Float32, self.right_motor_callback)

    self.obstacles = np.array([])
    self.min_obstacle_dist1 = 0
    
    self.front_left_sensor_dist = 200 
    self.front_right_sensor_dist = 200
    self.left_sensor_dist = 200
    self.right_sensor_dist = 200
    
  def front_left_sensor_callback(self, data):
    self.front_left_sensor_dist = data.data
    
  def front_right_sensor_callback(self, data):
    self.front_right_sensor_dist = data.data
    
  def left_sensor_callback(self, data):
    self.left_sensor_dist = data.data
    
  def right_sensor_callback(self, data):
    self.right_sensor_dist = data.data
    
  def left_motor_callback(self, data):
    self.left_motor_speed = data.data
    
  def right_motor_callback(self, data):
    self.right_motor_speed = data.data

  def drive(self):
    self.left_motor_control.set_motor_speed(0.3)
    self.right_motor_control.set_motor_speed(-0.3)
    rospy.sleep(4.5)
    self.left_motor_control.set_motor_speed(0)
    self.right_motor_control.set_motor_speed(0)

  def obstacle_avoidance(self):
    if self.front_sensor_dist < 5:
      self.left_motor_control.set_motor_speed(0, self.left_motor_speed)
      self.right_motor_control.set_motor_speed(0, self.right_motor_speed)
      print("stopped")
      
  def obstacle_detect(self):
    #sensor order: 1 front sensor on left, 2 front sensor on right, 3 right, 4 back, 5 left
    x = 0
    y = 0
    th = 0
    # print(self.front_left_sensor_dist)
    if(self.front_left_sensor_dist<5):
        x1 = x + self.front_left_sensor_dist*np.cos(th) 
        y1 = y + self.front_left_sensor_dist*np.sin(th)
        # self.min_obstacle_dist1 = np.min(np.sqrt((x1-self.obstacles[:,0])**2+(y1-self.obstacles[:,1])**2))
        # print(min_obstacle_dist1)
    # if(self.min_obstacle_dist1>=5):
        self.obstacles = np.append(self.obstacles, [x1,y1])
        print(self.obstacles)
        
  def front_localisation(self):
    Kp = 0.02
    diff = abs(self.front_left_sensor_dist - self.front_right_sensor_dist)
    if(diff>0.75):
      #dist1 left dist 2 right
      if(self.front_left_sensor_dist>self.front_right_sensor_dist):
          print('Turning Right')
          self.left_motor_control.set_motor_speed(Kp * diff)
          self.right_motor_control.set_motor_speed(-Kp * diff)
      elif(self.front_right_sensor_dist>self.front_left_sensor_dist):
          print('Turning Left')
          self.left_motor_control.set_motor_speed(-Kp * diff)
          self.right_motor_control.set_motor_speed(Kp * diff)
      print(diff)
    else:
      self.left_motor_control.set_motor_speed(0)
      self.right_motor_control.set_motor_speed(0)
      print("stopped")
      rospy.sleep(1)
      
    
      
if __name__ == "__main__":
  rospy.init_node('system')
  robot = System()
  rospy.sleep(1)
  robot.drive()
  rospy.spin()
  # while not rospy.is_shutdown():
  #   try:
  #     robot.drive()
  #   except rospy.ROSInterruptException:
  #     break