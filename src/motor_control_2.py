#! /usr/bin/env python
from threading import Thread
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep
from pi_controller import PIController
from encoder import Encoder
import rospy
from std_msgs.msg import Float32, Bool

class MotorControl: 
    def __init__(self, pin_PWM1, pin_PWM2, pin_EN, speed_control_kp, speed_control_ki, encoder_name): 
        self.motor_PWM1 = PWMOutputDevice(pin = pin_PWM1, initial_value = 0.0, frequency = 1000)
        self.motor_PWM2 = PWMOutputDevice(pin = pin_PWM2, initial_value = 0, frequency = 1000)
        self.motor_EN = DigitalOutputDevice(pin = pin_EN)

        # self.encoder = Encoder(pin_encoder_A, pin_encoder_B, encoder_name)
        
        self.encoder_name = encoder_name

        self.pi_controller = PIController(speed_control_kp, speed_control_ki, 0, 1, 0)
        
        self.encoder_sub = rospy.Subscriber('/' + encoder_name, Float32, self.motor_cb)
        self.set_motor_speed_sub = rospy.Subscriber('/set_' + encoder_name + '_speed', Float32, self.set_motor_speed_cb)
        self.turning_sub = rospy.Subscriber('/turning', Bool, queue_size=1)
        
        
        self.current_motor_speed = 0
        self.ref_motor_speed = 0
        
        self.turning = False
        
    def turning_cb(self, data):
      self.turning = data.data
    
    def motor_cb(self, data):
      self.motor_speed = data.data
        
    def set_motor_speed_cb(self, data):
      self.ref_motor_speed = data.data

    def enable_motor(self): 
      self.motor_EN.on()
        
    def disable_motor(self): 
      self.motor_EN.off()

    def set_motor_speed(self):

        # for idx in range(5): 
        #     # print('ref motor speed: ', self.ref_motor_speed, '    motor: ', self.encoder_name)
        #     if self.ref_motor_speed > 0: 
        #         self.motor_PWM1.value = self.pi_controller.update(self.ref_motor_speed, self.current_motor_speed)
        #         # print("ref motor speed: ", self.ref_motor_speed, "current motor speed: ", current_motor_speed)
        #         self.motor_PWM2.value = 0
        #     elif self.ref_motor_speed < 0: 
        #         self.motor_PWM1.value = 0
        #         # self.motor_PWM2.value = self.ref_motor_speed
        #         self.motor_PWM2.value = self.pi_controller.update(-self.ref_motor_speed, -self.current_motor_speed)
        #     else: 
        #         self.motor_PWM1.value = 0
        #         self.motor_PWM2.value = 0

        # print(f'Current Motor speed: {current_motor_speed} rev/s, Set Motor speed: {ref_motor_speed} rev/s'

                 # print('ref motor speed: ', self.ref_motor_speed, '    motor: ', self.encoder_name)
            self.ref_motor_speed = 0.2 if self.ref_motor_speed > 0.2 else self.ref_motor_speed
            if self.ref_motor_speed > 0: 
                # self.motor_PWM1.value = self.ref_motor_speed
                self.motor_PWM1.value = self.pi_controller.update(self.ref_motor_speed, self.current_motor_speed)
                # print("ref motor speed: ", self.ref_motor_speed, "current motor speed: ", self.motor_speed)
                self.motor_PWM2.value = 0
            elif self.ref_motor_speed < 0: 
                self.motor_PWM1.value = 0
                # self.motor_PWM2.value = -self.ref_motor_speed
                self.motor_PWM2.value = self.pi_controller.update(-self.ref_motor_speed, -self.current_motor_speed)
                # print("ref motor speed: ", self.ref_motor_speed, "current motor speed: ", self.motor_speed)
                
            else: 
                self.motor_PWM1.value = 0
                self.motor_PWM2.value = 0
                
    def set_motor_speed_turning(self):

        # for idx in range(5): 
        #     # print('ref motor speed: ', self.ref_motor_speed, '    motor: ', self.encoder_name)
        #     if self.ref_motor_speed > 0: 
        #         self.motor_PWM1.value = self.pi_controller.update(self.ref_motor_speed, self.current_motor_speed)
        #         # print("ref motor speed: ", self.ref_motor_speed, "current motor speed: ", current_motor_speed)
        #         self.motor_PWM2.value = 0
        #     elif self.ref_motor_speed < 0: 
        #         self.motor_PWM1.value = 0
        #         # self.motor_PWM2.value = self.ref_motor_speed
        #         self.motor_PWM2.value = self.pi_controller.update(-self.ref_motor_speed, -self.current_motor_speed)
        #     else: 
        #         self.motor_PWM1.value = 0
        #         self.motor_PWM2.value = 0

        # print(f'Current Motor speed: {current_motor_speed} rev/s, Set Motor speed: {ref_motor_speed} rev/s'

                 # print('ref motor speed: ', self.ref_motor_speed, '    motor: ', self.encoder_name)
            self.ref_motor_speed = 0.5 if self.ref_motor_speed > 0.5 else self.ref_motor_speed
            if self.ref_motor_speed > 0: 
                self.motor_PWM1.value = self.ref_motor_speed
                # self.motor_PWM1.value = self.pi_controller.update(self.ref_motor_speed, self.current_motor_speed)
                # print("ref motor speed: ", self.ref_motor_speed, "current motor speed: ", self.motor_speed)
                self.motor_PWM2.value = 0
            elif self.ref_motor_speed < 0: 
                self.motor_PWM1.value = 0
                self.motor_PWM2.value = -self.ref_motor_speed
                # self.motor_PWM2.value = self.pi_controller.update(-self.ref_motor_speed, -self.current_motor_speed)
                # print("ref motor speed: ", self.ref_motor_speed, "current motor speed: ", self.motor_speed)
                
            else: 
                self.motor_PWM1.value = 0
                self.motor_PWM2.value = 0
                
    def is_turning(self):
        return self.turning


if __name__ == "__main__":
    rospy.init_node('motor_control_2')
    
    print('Initialising right motor')
    pin_right_motor_PWM1 = 4
    pin_right_motor_PWM2 = 17
    pin_right_motor_EN = 27
    
    right_motor_speed_control_kp = 1
    right_motor_speed_control_ki = 0.1
    right_motor_name = 'right_motor'
    right_motor_control = MotorControl(pin_right_motor_PWM1, pin_right_motor_PWM2, pin_right_motor_EN, right_motor_speed_control_kp, right_motor_speed_control_ki, right_motor_name)
    right_motor_control.enable_motor()
    
    
    
    while not rospy.is_shutdown():
        try:
            # p1 = Thread(target = left_motor_control.set_motor_speed)
            # p2 = Thread(target = right_motor_control.set_motor_speed)
            # p1.start()
            # p2.start()

            # p1.join()
            # p2.join()
            # turning = right_motor_control.is_turning()
            # if turning:
            #     right_motor_control.set_motor_speed_turning()
            # else:
            right_motor_control.set_motor_speed_turning()
                
        except rospy.ROSInterruptException:
            break