#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
import math

pub = rospy.Publisher('monitor/event', Int32MultiArray, queue_size=10)

def callback_lidar(data):
    dist_threshold = 0.75
    nothing_close = True
    for i in range(len(data.ranges)):
        if(data.ranges[i] < dist_threshold):
            nothing_close = False
    msg = Int32MultiArray()
    msg.data.append(1000) # should be a hash)
    if(nothing_close):
       msg.data.append(1)
    else:
       msg.data.append(0)
    pub.publish(msg)

def safety_switch():

    rospy.init_node('lidar_monitor', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    safety_switch()
