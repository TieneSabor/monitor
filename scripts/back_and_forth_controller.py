#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def fake_controller():
    pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('fake_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 30
    while not rospy.is_shutdown():
        hello_twist = Twist()
        if (count % 120) < 60:
            hello_twist.linear.x = -0.35
        else:
           hello_twist.linear.x = 0.35
        rospy.loginfo(hello_twist)
        pub.publish(hello_twist)
        count = count + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_controller()
    except rospy.ROSInterruptException:
        pass

