from Adafruit_MotorHAT import Adafruit_MotorHAT # for DC + Stepper HAT
import rospy # python client library for ROS
from std_msgs.msg import Float64MultiArray # ros的message的資料類型
from math import pi, radians
class Tracking:
    def __init__(self):
        # node上運行此class, get the fully-qualified name of this node, 在此即為'tracking'
        self.node_name = rospy.get_name() 
        self.state = 1 # ?
        self.trig = None # ?
        # PWM controller, default address=0x60
        self.motorhat = Adafruit_MotorHAT(addr= 0x60)
        self.leftMotor     = self.motorhat.getMotor(1) # between 1-4
        self.rightMotor = self.motorhat.getMotor(2)
        self.right_pwm = 60
        self.left_pwm = 60
        self.leftMotor.setSpeed(self.left_pwm) # 0-255
        self.rightMotor.setSpeed(self.right_pwm)
        # subscriber去訂閱topic "/serial_node/odometry"(graph resource name of topic), 
        # topic的資料類型為std_msgs.msg.Float64MultiArray
        # 如果subscriber收到了msg, 則執行self.cbPosition Q: 因此cbPosition只能接受一個參數?
        self.subPosition=rospy.Subscriber("/serial_node/odometry",Float64MultiArray,self.cbPosition)
        self.FORWARD = 1
        self.BACKWARD = 2
        self.BRAKE = 3
        self.RELEASE = 4
        self.adjust_pwm = 5 # ?
        # 決定第一次進入stage的行為, 可能用不到
        self.in_stage1 = self.in_stage2 = self.in_stage3 = False
        rospy.on_shutdown(self.custom_shutdown)
        rospy.loginfo("[%s] Initialized!" %self.node_name)

    def cbPosition(self,msg):
        x = msg.data[0]
        y = msg.data[1]
        theta = msg.data[2]
        theta = theta % (2* pi)
        print(x,y,theta)

        if x <=0: # stright
            self.in_stage1 = True
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.left_pwm)
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)
        elif x >= 1 and theta < pi: # semi-circle
            if not self.in_stage2:
                self.in_stage2 = True
                self.right_pwm = 120
                self.left_pwm = 60 
            # 曲率太大, 右輪減速or左輪加速       
            if (x - 1) ** 2 + (y - 0.25) ** 2 < 0.25 ** 2: # left_pwm++ or right_pwm--
                self.right_pwm -= 2
            # 曲率太小, 右輪加速or左輪減速
            elif (x - 1) ** 2 + (y - 0.25) ** 2 > 0.25 ** 2: # left_pwm-- or right_pwm++
                self.left_pwm += 2
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.right_pwm)            
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)      
            self.right_pwm += 1
            self.left_pwm += 1                              
        elif x > 0 and y > 0: # theta > pi, 走直線            
            if theta - pi > 0: # left_pwm++ or right_pwm--            
                self.right_pwm -= 2
            elif theta - pi < 0: # left_pwm-- or right_pwm++
                self.left_pwm -= 2
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.right_pwm)
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)
            self.right_pwm += 1
            self.left_pwm += 1            
        else: # 停車
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
    rospy.init_node('tracking', anonymous = False) # 初始化node, node名稱為tracking
    Track = Tracking() # 裡
    rospy.spin() # keeps python from exiting unitil this node is stopped
