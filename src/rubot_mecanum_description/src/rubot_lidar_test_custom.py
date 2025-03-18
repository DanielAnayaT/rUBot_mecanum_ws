#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    laser_beams = len(msg.ranges)
    laser_factor = int(laser_beams / 360)

    print("Number of scan points: " + str(laser_beams))

    print("Distance at 0deg: " + str(msg.ranges[0]))
    print("Distance at 90deg: " + str(msg.ranges[90 * laser_factor]))
    print("Distance at 180deg: " + str(msg.ranges[180 * laser_factor]))
    print("Distance at 270deg: " + str(msg.ranges[270 * laser_factor]))
    print("Distance at 360deg: " + str(msg.ranges[laser_beams - 1]))

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
