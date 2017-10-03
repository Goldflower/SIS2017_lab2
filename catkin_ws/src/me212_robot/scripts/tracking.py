#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT # for DC + Stepper HAT
import rospy # python client library for ROS
from std_msgs.msg import Float64MultiArray # data type of message
from math import pi, radians

class Tracking:
    def __init__(self):
        self.initial_wheel_pwm = 60
        # node class, get the fully-qualified name of this node, i.e. 'tracking'
        self.node_name = rospy.get_name() 
        self.state = 1 # ?
        self.trig = None # ?
        # PWM controller, default address=0x60
        self.motorhat = Adafruit_MotorHAT(addr= 0x60)
        self.leftMotor     = self.motorhat.getMotor(1) # between 1-4
        self.rightMotor = self.motorhat.getMotor(2)
        self.right_pwm = self.initial_wheel_pwm
        self.left_pwm = self.initial_wheel_pwm
        self.leftMotor.setSpeed(self.left_pwm) # 0-255
        self.rightMotor.setSpeed(self.right_pwm)
        # subscriber subscribe topic "/serial_node/odometry"(graph resource name of topic), 
        # message type of topic is std_msgs.msg.Float64MultiArray
        # if subscriber received msg, execute self.cbPosition Q: so cbPosition only receive 1 parameter?
        self.subPosition=rospy.Subscriber("/serial_node/odometry",Float64MultiArray,self.cbPosition)
        self.FORWARD = 1
        self.BACKWARD = 2
        self.BRAKE = 3
        self.RELEASE = 4
        self.adjust_pwm = 5 # ?
        
        self.count_in_stage1 = self.count_in_stage2 = self.count_in_stage3 = 0
        self.in_stage1 = self.in_stage2 = self.in_stage3 = False
        rospy.on_shutdown(self.custom_shutdown)
        rospy.loginfo("[%s] Initialized!" %self.node_name)

    def cbPosition(self,msg):
        x = msg.data[0]
        y = msg.data[1]
        theta = msg.data[2]
        theta = theta % (2 * pi)
        print(x,y,theta)

        if x <= 1 and y < 0.1: # stright
            if theta > 0: # left_pwm++ or right_pwm--
                self.right_pwm -= 3
            elif theta < 0:
                self.left_pwm -= 3
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.left_pwm)
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)
        elif x >= 1 and theta < pi: # semi-circle
            if self.count_in_stage2 == 0: 
                self.count_in_stage2 += 1
                self.leftMotor.run(self.RELEASE)
                self.rightMotor.run(self.RELEASE)
                self.right_pwm = 2 * self.initial_wheel_pwm
                self.left_pwm = self.initial_wheel_pwm                
            elif self.count_in_stage2 < 3: 
                self.count_in_stage2 += 1
                self.right_pwm = 2 * self.initial_wheel_pwm
                self.left_pwm = self.initial_wheel_pwm
            elif (x - 1) ** 2 + (y - 0.25) ** 2 < 0.25 ** 2: # left_pwm++ or right_pwm--
                self.right_pwm -= 3
            elif (x - 1) ** 2 + (y - 0.25) ** 2 > 0.25 ** 2: # left_pwm-- or right_pwm++
                self.left_pwm -= 3
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.right_pwm)            
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD) 
            self.right_pwm = 2 * self.initial_wheel_pwm
            self.left_pwm = self.initial_wheel_pwm                                    
        elif x > 0 and theta >= pi: # theta > pi
            if self.count_in_stage3 == 0:            
                self.count_in_stage3 += 1
                self.leftMotor.run(self.RELEASE)
                self.rightMotor.run(self.RELEASE)                
                self.right_pwm = self.initial_wheel_pwm
                self.left_pwm = self.initial_wheel_pwm
            elif self.count_in_stage3 < 3:                  
                self.count_in_stage2 += 1
                self.right_pwm = self.initial_wheel_pwm
                self.left_pwm = self.initial_wheel_pwm                
            elif theta - pi > 0: # left_pwm++ or right_pwm--            
                self.right_pwm -= 3
            elif theta - pi < 0: # left_pwm-- or right_pwm++
                self.left_pwm -= 3
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.right_pwm)
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)
            self.right_pwm = self.initial_wheel_pwm
            self.left_pwm = self.initial_wheel_pwm
        else:
            self.leftMotor.run(self.RELEASE)
            self.rightMotor.run(self.RELEASE)
            

        # stages: 1) straight line,
        #         2) semi-circle
        #         3) straight line again.
            

    def custom_shutdown(self):
        self.leftMotor.run(4)
        self.rightMotor.run(4)
        del self.motorhat

if __name__ == '__main__':
    rospy.init_node('tracking', anonymous = False) # initialize node, name of node is tracking
    Track = Tracking() 
    rospy.spin() # keeps python from exiting unitil this node is stopped
