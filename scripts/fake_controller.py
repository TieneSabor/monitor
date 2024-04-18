#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def fake_controller():
    pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('fake_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_twist = Twist()
        hello_twist.linear.x = 0.25
        rospy.loginfo(hello_twist)
        pub.publish(hello_twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_controller()
    except rospy.ROSInterruptException:
        pass

