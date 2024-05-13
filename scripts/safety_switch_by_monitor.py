#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

enable = False
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
can_move = True

def callback_listener(data):
    # if not enable or not can_move :
    if not enable :
        empty_twist = Twist()
        pub.publish(empty_twist)  
    elif not can_move :
        print("Violated")
    else:
        pub.publish(data)
        
def safe_callback_listener(data):
    if not can_move and enable:
        pub.publish(data)

def callback_joy(data):
    # If the right trigger is pressed, and we are reading nonzero values
    # the bluetooth controller is being held down by an operator.
    global enable
    if (data.axes[5] < 0.95 and not (data.axes[2]>-0.001 and data.axes[2]<0.001)):
    	enable = True
    else:
    	enable = False

def callback_monitor(data):
    # For now, assume the 1st field is a hash
    # and the second field is the lidar monitor status
    global can_move
    STATUS_META_HASH = 2000
    if(data.data[0] == STATUS_META_HASH and data.data[1] == 0):
        can_move = True
    else:
        can_move = False

def safety_switch():

    rospy.init_node('safety_switch_by_monitor', anonymous=True)
    rospy.Subscriber("/controller/cmd_vel", Twist, callback_listener)
    rospy.Subscriber("/controller/cmd_vel_safe", Twist, safe_callback_listener)

    rospy.Subscriber("/bluetooth_teleop/joy", Joy, callback_joy)
    rospy.Subscriber("/monitor/status", Int32MultiArray, callback_monitor)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    safety_switch()
