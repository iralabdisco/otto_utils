#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def map_from_to(value, start1, stop1, start2, stop2):
    y = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1))
    return y

def callback(data):
    global max_linear
    global max_angular

    twist = Twist()

    # If trigger is pressed, publish the velocity
    if data.buttons[5] == 1 and data.buttons[7] == 1:
        linear_vel = map_from_to(data.axes[2], -1, 1, -max_linear, max_linear)
        angular_vel = map_from_to(data.axes[0], -1, 1, -max_angular, max_angular)
    else:
        linear_vel = 0
        angular_vel = 0

    # If stop button pressed, stop the robot
    if data.buttons[0] == 1:
        linear_vel = 0
        angular_vel = 0

    twist.linear.x = linear_vel
    twist.angular.z = angular_vel

    pub.publish(twist)

def start():
    global pub
    global max_linear
    global max_angular
    rospy.init_node('joy_to_cmd_vel')

    max_linear = rospy.get_param("~max_linear", 0.4)
    max_angular = rospy.get_param("~max_angular", 1)

    print("Starting joypad with max_linear: " + str(max_linear) + " and max_angular: " + str(max_angular))

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    start()

