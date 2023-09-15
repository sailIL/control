PKG = 'control'
import rospy
import roslib; roslib.load_manifest(PKG)
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import serial

"class state_machine(object):"
#     def __init__(self):
        
#         self.right_servo = "right_servo"
#         self.left_servo = "left_servo"
#         self.x = 0
#         self.y = 0
#         self.angle = 0
#         self.dist=0

#         while not rospy.is_shutdown():
#             rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
#             try :
#                 self.Situation()
#             except KeyboardInterrupt :
#                 self.move(self.right_thruster, 90)  # Stop propeller=90
#                 self.move(self.left_thruster, 90)
#                 self.move(self.bow_thruster, 90) 
            
#     def cmd_callback(self, cmd_data):
#         self.x = cmd_data.linear.x
#         if self.x == None :
#             self.x = 0
#         self.y = cmd_data.linear.y
#         if self.y == None :
#             self.y = 0
#         self.angle = math.degrees(cmd_data.angular.z)
#         if self.angle == None :
#             self.angle = 0
#         self.dist = math.sqrt((self.x)**2 + (self.y)**2)
#         if self.angle == 0 :
#             try :
#                 if self.y !=0 :
#                     self.angle = math.degrees(math.atan2(self.y,self.x))
#                 else:
#                     self.angle = 0
#             except ZeroDivisionError :
#                 if self.y > 0 :
#                     self.angle = 90
#                 elif self.y < 0 :
#                     self.angle = -90

class PID:
    def __init__(self):
        # arduino initialize
        self.arduino = serial.Serial(port='/dev/ttyUSB0' , baudrate = 9600, timeout = 1) #Communication with Arduino
        self.right_thruster = "right_thruster"
        self.left_thruster = "left_thruster"
        self.bow_thruster = "bow_thruster"
        # initialize node and clock
        rospy.init_node('yoav_PID')
        self.init_current_time = time.time()
        # PID parameters
        self.kp = 0.6
        self.kd = 0.125
        self.kp_angluar = 0.2
        self.kd_angular = 0.4
        self.bias = -0.01
        self.distance_iteration_time = 0
        self.angular_iteration_time = 0

        # PWM signal parameters
        self.servo_value = 0
        self.min_servo_value = 0
        self.mid_servo_value = 90
        self.max_servo_value = 180
        self.servo_value_range = self.max_servo_value - self.min_servo_value

        # PID variables
        self.yaw_last_time = 0
        self.distance_last_time = 0
        self.yaw_current_time = 0
        self.distance_current_time = 0
        self.previous_distance = 0
        self.distance = 0
        self.previous_angle_change = 0
        self.heading_error = 0

        # sevro valued we'll send
        self.linear_thrust = 0
        self.angular_thrust = 0

        # while not rospy.is_shutdown(): # odom sub gives us location, yaw sub gives relative angle to first coordinate axis, distance sub gives us distance from target 
        self.odom_sub = rospy.Subscriber('/odometry', Odometry, self.odom_callback)
        self.yaw_angle_sub = rospy.Subscriber('/angle', Float64, self.yaw_callback)
        self.raz_sub = rospy.Subscriber('/cmd_vel', Twist, self.distance_callback)
            

    def odom_callback(self, data):
        self.odom_data = data.pose.pose

    def yaw_callback(self, data):
        self.right_hand_yaw = data #get the current yaw, if needed take it from odometry and use quaternium to euler
        self.yaw_current_time = time.time()

    def distance_callback(self, raz_data): # Target position
        self.target_pos = raz_data
        self.target_pos.x = raz_data.linear.x
        self.target_pos.y = raz_data.linear.y
        self.target_pos.z = raz_data.angular.z
        self.distance_current_time = time.time()
        
    def distance(self): # Function to calculate distance between two points, and angle change between wanted and current angle
        self.distance = math.sqrt((self.target_pos.x)**2 + (self.target_pos.y)**2)
        current_heading = self.right_hand_yaw
        self.heading_error = self.target_pos.z - current_heading
    
    def calculate_servo_value(self): # Function to calculate PWM signal based on position error
        if self.distance_last_time == 0:
            self.distance_iteration_time = self.distance_current_time - self.init_current_time # dt distance
        else:
            self.distance_iteration_time  = self.distance_current_time - self.distance_last_time
        derivative_distance = (self.distance - self.previous_distance) / self.distance_iteration_time
        self.servo_value = self.kp*self.distance + self.kd*derivative_distance
        if self.servo_value > 180:
            self.servo_value = 180
        elif self.servo_value < 0:
            self.servo_value = 0
        self.previous_distance = self.distance

        self.linear_thrust = self.servo_value

        time.sleep(0.1) # Wait for next loop iteration

    def calculate_heading_error(self): # Function to calculate heading error
        if self.yaw_last_time == 0:
            self.angular_iteration_time = self.yaw_current_time - self.init_current_time
        else:
            self.angular_iteration_time = self.yaw_current_time - self.yaw_last_time
        derivative_angle = (self.heading_error - self.previous_angle_change) / self.angular_iteration_time
        self.bow_servo_value = self.kp_angluar*self.heading_error + self.kd_angular*derivative_angle
        if self.bow_servo_value > 180:
            self.bow_servo_value -= 180
        elif self.bow_servo_value < -180:
            self.bow_servo_value += 180
        self.yaw_current_time = self.yaw_last_time

        self.angular_thrust = self.bow_servo_value

    def move(self, motor):
        i=0
        if motor == "right_thruster":
            i = 1
            angle = self.linear_thrust
        elif motor == "left_thruster":
            i = 2
            angle = self.linear_thrust
        elif motor == "bow_thruster":
            i = 3
            angle = self.angular_thrust
        message = str(angle + 1000 * i )  #the i*1000 is for decoding the motor we want to move via the send message.
        print("motor : "+str(motor)+ " ; angle value "+str(angle) +" ! ")
        self.arduino.write(bytes(message).encode("utf-8"))

        time.sleep(0.05)  # in order to make sure the arduino will read different messages as different messages
    
    def printer(self):
        time.sleep(1)
        print("servo_value: ", self.linear_thrust)
        print("bow_thruster value: ", self.bow_servo_value )

if __name__ == '__main__':
    Subscriber = PID()
    rate = rospy.Rate(50) # 10 Hz
    while not rospy.is_shutdown():
        Subscriber.move()
        rate.sleep()
