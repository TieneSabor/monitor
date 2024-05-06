#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool

pub = rospy.Publisher('monitor/event', Int32MultiArray, queue_size=10)

# Format of the multiarray:
# 0: Format code. 1001 for the format below
# 1: Surrounding area is clear (no objects within 0.75m)
# 2: Robot within work zone (pose within 2m of origin)
# 3: Lidar is live (last message no more than 2seconds old)
# 4: Robot position is live (last message no more than 5seconds old)
# 5: Robot position is at X1 (within 0.2m)
# 6: Robot position is at X2 (within 0.2m)


# Atomic proposition values and time of receipt.
# 0 for false, 1 for true, -1 for unknown.
# Not thread safe!
area_is_clear = -1
within_work_zone = -1
lidar_is_live = -1
robot_position_is_live = -1
robot_is_at_x1 = -1
robot_is_at_x2 = -1

# lidar_time = -1
# zone_time = -1
# x1_time = -1
# x2_time = -1


def callback_lidar_prop(data):
    global area_is_clear
    if data.data:
        area_is_clear = 1
    else:
        area_is_clear = 0
    # update time

def callback_zone_prop(data):
    global within_work_zone
    if data.data:
        within_work_zone = 1
    else:
        within_work_zone = 0

def callback_x1_prop(data):
    global robot_is_at_x1
    if data.data:
        robot_is_at_x1 = 1
    else:
        robot_is_at_x1 = 0

def callback_x2_prop(data):
    global robot_is_at_x2
    if data.data:
        robot_is_at_x2 = 1
    else:
        robot_is_at_x2 = 0


def instrument_fusion():
    rospy.init_node('instrument_fusion', anonymous=True)
    rospy.Subscriber("/monitor/event/lidar", Bool, callback_lidar_prop)
    rospy.Subscriber("/monitor/event/isinzone", Bool, callback_zone_prop)
    rospy.Subscriber("/monitor/event/isatx1", Bool, callback_x1_prop)
    rospy.Subscriber("/monitor/event/isatx2", Bool, callback_x2_prop)

    # publish
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # update lidar_is_live, pose_is_live
        msg = Int32MultiArray()
        msg.data.append(1001)
        msg.data.append(area_is_clear)
        msg.data.append(within_work_zone)
        msg.data.append(lidar_is_live)
        msg.data.append(robot_position_is_live)
        msg.data.append(robot_is_at_x1)
        msg.data.append(robot_is_at_x2)

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        instrument_fusion()
    except rospy.ROSInterruptException:
        pass

