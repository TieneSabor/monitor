#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import math

pub_zone = rospy.Publisher('monitor/event/isinzone', Bool, queue_size=10)
pub_x1 = rospy.Publisher('monitor/event/isatx1', Bool, queue_size=10)
pub_x2 = rospy.Publisher('monitor/event/isatx2', Bool, queue_size=10)

def callback_pose(data):
    # Stay within 2m of the origin. Robot moves in XZ plane
    zone_threshold = 1.5
    target_threshold = 0.2
    within_work_zone = close_to(0,0,data.pose.position.x,data.pose.position.z,zone_threshold)
    is_at_x1 = close_to(1,0,data.pose.position.x,data.pose.position.z,target_threshold)
    is_at_x2 = close_to(-1,0,data.pose.position.x,data.pose.position.z,target_threshold)

    pub_zone.publish(within_work_zone)
    pub_x1.publish(is_at_x1)
    pub_x2.publish(is_at_x2)

def close_to(target_x, target_z, actual_x, actual_z, dist_threshold):
    x_dist = target_x - actual_x
    z_dist = target_z - actual_z
    distance_sq = x_dist*x_dist + z_dist*z_dist
    is_close = distance_sq < dist_threshold * dist_threshold
    return is_close

def pose_monitor():
    rospy.init_node('pose_monitor', anonymous=True)
    rospy.Subscriber("/mocap_node/jackal7/pose", PoseStamped, callback_pose)
    rospy.spin()

if __name__ == '__main__':
    pose_monitor()
