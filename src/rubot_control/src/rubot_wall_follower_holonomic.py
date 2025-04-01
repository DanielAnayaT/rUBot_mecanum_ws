#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=False)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown_callback)
        self.rate = rospy.Rate(25)

        self.d = rospy.get_param("~distance_laser", 0.3)
        self.vf = rospy.get_param("~speed_factor", 1.0)
        self.vx = rospy.get_param("~forward_speed", 0.2) * self.vf
        self.vy = rospy.get_param("~lateral_speed", 0.2) * self.vf
        self.wz = rospy.get_param("~rotation_speed", 1.0) * self.vf

        self.isScanRangesLengthCorrectionFactorCalculated = False
        self.scanRangesLengthCorrectionFactor = 2

        self.shutting_down = False
        self.regions = None
        self.msg = None

    def get_distance(self, msg, minAngle, maxAngle):
        minAngle = int(minAngle * self.scanRangesLengthCorrectionFactor)
        maxAngle = int(maxAngle * self.scanRangesLengthCorrectionFactor)
        return min(min(msg.ranges[minAngle:maxAngle]), 3)

    def scan_callback(self, msg):
        self.msg = msg

    def set_regions(self):
        if self.shutting_down or self.msg is None:
            return

        if not self.isScanRangesLengthCorrectionFactorCalculated:
            self.scanRangesLengthCorrectionFactor = len(self.msg.ranges) / 360
            self.isScanRangesLengthCorrectionFactorCalculated = True

        self.regions = {
            'rback': self.get_distance(self.msg, 0, 30),
            'bright': self.get_distance(self.msg, 30, 90),
            'right': self.get_distance(self.msg, 90, 120),
            'fright': self.get_distance(self.msg, 120, 170),
            'front': self.get_distance(self.msg, 170, 190),
            'fleft': self.get_distance(self.msg, 190, 240),
            'left': self.get_distance(self.msg, 240, 270),
            'bleft': self.get_distance(self.msg, 270, 330),
            'lback': self.get_distance(self.msg, 330, 360),
        }

        self.regions["back"] = min(self.regions["rback"], self.regions["lback"])

    def run(self):
        while not rospy.is_shutdown():
            self.set_regions()
            if self.regions:
                self.take_action(self.regions)

    def take_action(self, regions):
        msg = Twist()
        linear_x = 0
        linear_y = 0
        angular_z = 0

        state_description = ''

        if regions['front'] > self.d and regions['fright'] > 2 * self.d and regions['right'] > 2 * self.d and regions['bright'] > 2 * self.d:
            state_description = 'starting'
            linear_x = self.vx
            angular_z = 0

        elif regions['front'] < self.d and regions['left'] > self.d:
            state_description = 'front - move sideways'
            linear_y = self.vy

        elif regions['back'] < self.d and regions['right'] > self.d:
            state_description = 'back - move sideways'
            linear_y = -self.vy

        elif regions['front'] < self.d:
            state_description = 'front'
            linear_x = 0
            linear_y = 0
            angular_z = self.wz

        elif regions['fright'] < self.d and regions['right'] > self.d:
            state_description = 'fright'
            linear_x = 0
            angular_z = self.wz

        elif regions['right'] < (self.d + 0.1):  # and regions['bright'] > self.d:
            state_description = 'right'
            linear_x = self.vx
            linear_y = 0
            angular_z = 0

        elif regions['bright'] < self.d:
            state_description = 'bright'
            linear_x = 0
            angular_z = -2 * self.wz

        else:
            state_description = 'Far'
            linear_x = self.vx / 2
            linear_y = 0
            angular_z = -2 * self.wz

        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.pub.publish(msg)
        self.rate.sleep()

    def shutdown_callback(self):
        self.shutting_down = True
        self.sub.unregister()
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        self.pub.publish(msg)
        self.rate.sleep()
        rospy.loginfo("Stop rUBot")

if __name__ == '__main__':
    try:
        wall_follower = WallFollower()
        wall_follower.run()
    except rospy.ROSInterruptException:
        pass
