#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def fake_monitor_status():
    pub = rospy.Publisher('/monitor/status', Int32MultiArray, queue_size=10)
    rospy.init_node('fake_monitor_status', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Int32MultiArray()
        msg.data.append(2000) # should be a hash)
        msg.data.append(0)
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    fake_monitor_status()
