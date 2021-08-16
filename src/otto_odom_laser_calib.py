#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped
from otto_serial_driver.msg import otto_ticks, otto_pid_tuning_values
import math

ticks_per_revolution = 0

calib_pub = rospy.Publisher('/otto_odom_calib', Vector3Stamped, queue_size=10)
calib_msg = Vector3Stamped()

def otto_ticks_callback(data):
    global ticks_per_revolution

    left_ang_vel = ((data.left_ticks*2*math.pi)/ticks_per_revolution)/(data.delta_millis/1000)
    right_ang_vel = (data.right_ticks*2*math.pi)/ticks_per_revolution/(data.delta_millis/1000)
    
    calib_msg.header.stamp = data.timestamp
    calib_msg.vector.x = left_ang_vel
    calib_msg.vector.y = right_ang_vel
    calib_pub.publish(calib_msg)

def otto_odom_laser_calib():
    ## ROS init node
    rospy.init_node('otto_odom_laser_calib', anonymous=True, log_level=rospy.DEBUG)
    rate = rospy.Rate(10)

    ## get coniguration parameters
    global ticks_per_revolution

    ticks_per_revolution = rospy.get_param("ticks_per_revolution", 148000) 
    rospy.Subscriber('/otto_ticks', otto_ticks, otto_ticks_callback)
    
    while (not rospy.is_shutdown()):
        rospy.spin()


if __name__ == '__main__':
    otto_odom_laser_calib()
