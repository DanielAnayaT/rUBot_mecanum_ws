#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleNavigator:
    def __init__(self):
        rospy.init_node("simple_nav")

        # Parámetros configurables
        self.safe_distance = rospy.get_param("~safe_distance", 0.5)
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)

        # Publicador de velocidad
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Suscriptor al sensor LIDAR
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        rospy.on_shutdown(self.shutdown_callback)

        self.isScanRangesLengthCorrectionFactorCalculated = False
        self.scanRangesLengthCorrectionFactor = 2

        self.closest_front = float("inf")
        self.closest_back = float("inf")
        self.rate = rospy.Rate(10)  # 10 Hz

        self.shutting_down = False

    def get_distance(self, msg, minAngle, maxAngle):
        minAngle = int(minAngle * self.scanRangesLengthCorrectionFactor)
        maxAngle = int(maxAngle * self.scanRangesLengthCorrectionFactor)
        return min(min(msg.ranges[minAngle:maxAngle]), 3)

    def laser_callback(self, msg):
        if self.shutting_down:
            return

        if not self.isScanRangesLengthCorrectionFactorCalculated:
            self.scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            self.isScanRangesLengthCorrectionFactorCalculated = True

        regions = {
            'rback': self.get_distance(msg, 0, 30),
            'bright': self.get_distance(msg, 30, 90),
            'right': self.get_distance(msg, 90, 120),
            'fright': self.get_distance(msg, 120, 170),
            'front': self.get_distance(msg, 170, 190),
            'fleft': self.get_distance(msg, 190, 240),
            'left': self.get_distance(msg, 240, 270),
            'bleft': self.get_distance(msg, 270, 330),
            'lback': self.get_distance(msg, 330, 360),
        }

        self.closest_front = regions["front"]
        self.closest_back = min(regions["rback"], regions["lback"])

    def run(self):
        msg = Twist()
        while not rospy.is_shutdown():
            if self.closest_front < self.safe_distance:
                if self.closest_back < self.safe_distance:
                    # Si hay obstáculos tanto delante como detrás → girar en el sitio
                    msg.linear.x = 0
                    msg.angular.z = self.rotation_speed
                else:
                    # Si solo hay obstáculo delante → retroceder y girar
                    msg.linear.x = self.backward_speed
                    msg.angular.z = self.rotation_speed
            else:
                # Si no hay obstáculos delante → avanzar
                msg.linear.x = self.forward_speed
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

    def shutdown_callback(self):
        self.shutting_down = True
        self.sub.unregister()
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        self.cmd_vel_pub.publish(msg)
        self.rate.sleep()
        rospy.loginfo("Stop rUBot")


if __name__ == "__main__":
    node = SimpleNavigator()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
