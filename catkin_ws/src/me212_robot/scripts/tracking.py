from Adafruit_MotorHAT import Adafruit_MotorHAT # for DC + Stepper HAT
import rospy # python client library for ROS
from std_msgs.msg import Float64MultiArray # ros的message的資料類型
from math import pi, radians
class Tracking:
    def __init__(self):
        self.initial_wheel_pwm = 60
        # node上運行此class, get the fully-qualified name of this node, 在此即為'tracking'
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
        self.count_in_stage1 = self.count_in_stage2 = self.count_in_stage3 = 0
        rospy.on_shutdown(self.custom_shutdown)
        rospy.loginfo("[%s] Initialized!" %self.node_name)

    def cbPosition(self,msg):
        x = msg.data[0]
        y = msg.data[1]
        theta = msg.data[2]
        theta = theta % (2* pi)
        print(x,y,theta)

        if x <= 0: # stright
            if theta > 0: # left_pwm++ or right_pwm--
                self.right_pwm -= 3
            elif theta < 0:
                self.left_pwm -= 3
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.left_pwm)
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)
        elif x >= 1 and theta < pi: # semi-circle
            if self.count_in_stage2 == 0: # 不知道有沒有用的heuristic, 先調整到正確的pwm走一下
                self.count_in_stage2 += 1
                self.leftMotor.run(self.BRAKE)
                self.rightMotor.run(self.BRAKE)
                self.right_pwm = 2 * self.initial_wheel_pwm
                self.left_pwm = self.initial_wheel_pwm                
            elif self.count_in_stage2 < 3: 
                self.count_in_stage2 += 1
                self.right_pwm = 2 * self.initial_wheel_pwm
                self.left_pwm = self.initial_wheel_pwm
            # 曲率太大, 右輪減速or左輪加速       
            elif (x - 1) ** 2 + (y - 0.25) ** 2 < 0.25 ** 2: # left_pwm++ or right_pwm--
                self.right_pwm -= 3
            # 曲率太小, 右輪加速or左輪減速
            elif (x - 1) ** 2 + (y - 0.25) ** 2 > 0.25 ** 2: # left_pwm-- or right_pwm++
                self.left_pwm -= 3
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.right_pwm)            
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)                         
        elif x > 0 and y > 0: # theta > pi, 走直線            
            if theta - pi > 0: # left_pwm++ or right_pwm--            
                self.right_pwm -= 2
            elif theta - pi < 0: # left_pwm-- or right_pwm++
                self.left_pwm -= 2
            self.leftMotor.setSpeed(self.left_pwm)
            self.rightMotor.setSpeed(self.right_pwm)
            self.leftMotor.run(self.FORWARD)
            self.rightMotor.run(self.FORWARD)
        else: # 停車
            self.leftMotor.run(self.RELEASE)
            self.rightMotor.run(self.RELEASE)
        # 每次都回復初始值
        self.right_pwm = self.initial_wheel_pwm
        self.left_pwm = self.initial_wheel_pwm
            

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
