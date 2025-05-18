#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import time

class MoveDecisionNode:
    def __init__(self):
        rospy.init_node("keras_make_move_action")

        rospy.Subscriber("/predicted_class", String, self.class_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.current_class = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.signal_x = None
        self.signal_y = None
        self.detection_time = None
        self.detection_distance_threshold = 0.3  # metros
        self.detection_timeout = 10.0  # segundos

        rospy.loginfo("Nodo keras_make_move_action activo.")

    def class_callback(self, msg):
        self.current_class = msg.data
        self.signal_x = self.robot_x + 0.5  # asumimos señal 0.5m delante
        self.signal_y = self.robot_y
        self.detection_time = time.time()
        rospy.loginfo(f"Clase recibida: {self.current_class}")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        if self.current_class and self.signal_x and self.signal_y and self.detection_time:
            elapsed = time.time() - self.detection_time
            if elapsed < self.detection_timeout:
                dist = np.hypot(self.robot_x - self.signal_x, self.robot_y - self.signal_y)
                if dist <= self.detection_distance_threshold:
                    twist = Twist()
                    rospy.loginfo(f"A {dist:.2f}m de la señal '{self.current_class}'")

                    if self.current_class == "Stop":
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)

                    elif self.current_class == "Give_Way":
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)
                        rospy.sleep(5)  # esperar 5 segundos

                    elif self.current_class == "Turn_Left":
                        twist.angular.z = 0.5
                        self.cmd_vel_pub.publish(twist)

                    elif self.current_class == "Turn_Right":
                        twist.angular.z = -0.5
                        self.cmd_vel_pub.publish(twist)

                    # Evitar repetir acciones
                    self.current_class = None

if __name__ == "__main__":
    MoveDecisionNode()
    rospy.spin()
