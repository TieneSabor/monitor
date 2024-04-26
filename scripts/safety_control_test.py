#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# for loading data from matlab and calculate optimal control
from os.path import dirname, join as pjoin
import scipy.io as sio
from scipy.interpolate import LinearNDInterpolator

_MAX = 10000.0

# velocity limit
Vrange = [-1.0, 1.0]
Wrange = [-0.5, 0.5]
    
pub = rospy.Publisher('/controller/cmd_vel_safe', Twist, queue_size=10)

# import data
data_dir = pjoin(dirname(__file__), '..', 'data')
mat_fname = pjoin(data_dir, 'V_gradientV.mat')
V_gradientV = sio.loadmat(mat_fname)
# print(V_gradientV['V'])
gx_1d = [j for sub in V_gradientV['gx'] for j in sub]
gy_1d = [j for sub in V_gradientV['gy'] for j in sub]
DxV_1d = [j for sub in V_gradientV['DxV'] for j in sub]
DyV_1d = [j for sub in V_gradientV['DyV'] for j in sub]
interp_DxV = LinearNDInterpolator(list(zip(gx_1d, gy_1d)), DxV_1d)
interp_DyV = LinearNDInterpolator(list(zip(gx_1d, gy_1d)), DyV_1d)

def smoothStep(x):
    xbnd = 10
    xl = -xbnd
    xr = xbnd
    gl = 0
    gr = 1
    y = (1/(xr-xl))*x + 0.5
    g = (gr-gl)*(3*y**2 - 2*y**3) + ((gr-gl)/2 - 0.5)
    return g


def callback_lidar(data):
    # We can define the obstacle as the closest lidar point cloud 
    minRange = _MAX
    minRangeAng = 0.0
    nA = len(data.ranges)
    for i in range(len(data.ranges)):
        if(data.ranges[i] < minRange):
            minRange = _MAX
            minRangeAng = data.angle_min + (i/nA)*(data.angle_max - data.angle_min)
    obstacle_x = minRange*cos(minRangeAng)
    obstacle_y = minRange*sin(minRangeAng)

    # and we can calculate the "optimal" speed command
    px = interp_DxV(obstacle_x, obstacle_y)
    py = interp_DyV(obstacle_x, obstacle_y)
    vFlag = smoothStep(-px)
    wFlag = smoothStep(px*obstacle_y - py*obstacle_x)
    safe_twist = Twist()
    safe_twist.linear.x  = vFlag*Vrange[0] + (1-vFlag)*Vrange[1]
    safe_twist.angular.z = wFlag*Wrange[0] + (1-wFlag)*Wrange[1]
    rospy.loginfo(safe_twist)
    pub.publish(safe_twist)
    rate.sleep()

def safety_control_test():
    # prepare for ros
    rospy.init_node('safety_control', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_lidar)
    rospy.loginfo("Waiting for lidar data...")
    rospy.spin()

if __name__ == '__main__':
    try:
        safety_control_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception Occured")