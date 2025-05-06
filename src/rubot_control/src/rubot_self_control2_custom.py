#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleNavigator:
    def __init__(self):
        rospy.init_node("simple_nav")

        # Parámetros configurables
        self.safe_distance = rospy.get_param("~safe_distance", 0.4)
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.2)
        self.rotation_speed = rospy.get_param("~rotation_speed", 0.3)

        # Publicador de velocidad
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Suscriptor al sensor LIDAR
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.closest_front = float("inf")
        self.closest_back = float("inf")
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = 999  # Reemplazar valores infinitos por un número alto
        
        # Dividir el escaneo en sectores:
        # Frente = ±30º del eje delantero
        # Parte trasera = ±30º del eje trasero (180º)
        front_angles = np.concatenate((ranges[:30], ranges[-30:]))   # 30º a la izquierda y derecha del frente
        back_angles = ranges[150:210]  # 30º a la izquierda y derecha de 180º (parte trasera)

        self.closest_front = np.min(front_angles)
        self.closest_back = np.min(back_angles)

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

if __name__ == "__main__":
    node = SimpleNavigator()
    node.run()
