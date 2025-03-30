#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

pub = None
d = 0
vx = 0
wz = 0
vf = 0

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
        scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
        isScanRangesLengthCorrectionFactorCalculated = True

    # Define los nuevos rangos para left, fleft, bleft, ajustando los ángulos según sea necesario
    fleft_min = int(190 * scanRangesLengthCorrectionFactor)
    fleft_max = int(210 * scanRangesLengthCorrectionFactor)
    left_min = int(250 * scanRangesLengthCorrectionFactor)
    left_max = int(270 * scanRangesLengthCorrectionFactor)
    # Continúa con las definiciones anteriores
    bright_min = int(30 * scanRangesLengthCorrectionFactor)
    bright_max = int(90 * scanRangesLengthCorrectionFactor)
    right_min = int(90 * scanRangesLengthCorrectionFactor)
    right_max = int(120 * scanRangesLengthCorrectionFactor)
    fright_min = int(120 * scanRangesLengthCorrectionFactor)
    fright_max = int(170 * scanRangesLengthCorrectionFactor)
    front_min= int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)

    regions = {
        'fleft': min(min(msg.ranges[fleft_min:fleft_max]), 3),
        'left': min(min(msg.ranges[left_min:left_max]), 3),
        'bright':  min(min(msg.ranges[bright_min:bright_max]), 3),
        'right':  min(min(msg.ranges[right_min:right_max]), 3),
        'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
    }

    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    linear_y = 0

    state_description = ''

    if regions['front'] > d and all(regions[direction] > 2*d for direction in ['fright', 'right', 'bright', 'fleft', 'left']):
        state_description = 'case 1 - nothing'
        linear_x = vx
        angular_z = 0
    elif regions['front'] < d:
        state_description = 'case 2 - front'
        linear_x = 0
        linear_y=vx
        angular_z = wz/2 # turn left
    elif regions['fright'] < d:
        state_description = 'case 3 - fright'
        linear_x = vx # turn left slow
        linear_y = vx/2
        angular_z = wz
    elif regions['front'] > d and regions['right'] < d:
        state_description = 'case 4 - right'
        linear_x = vx # straight
        linear_y = 0
        angular_z = -wz/2
    elif regions['bright'] < d:
        state_description = 'case 5 - bright'
        linear_x = vx/4 # turn right (return to wall)
        angular_z = -wz/2
        linear_y = -vx/2
    elif regions['fleft'] < d:
        state_description = 'case 7 - fleft'
        linear_x = -vx/2  
        angular_z = -wz/2  
        linear_y = -vx/2
    elif regions['left'] < d:
        state_description = 'case 8 - left'
        linear_x = vx/3
        linear_y = -vx/3
        angular_z = wz*1.5
    else:
        state_description = 'case 6 - Far'
        linear_x = vx/2 # turn right more
        angular_z = -wz/2
        linear_y = 0

    rospy.loginfo(state_description)
    msg.linear.x = linear_x * vf
    msg.angular.z = angular_z * vf
    msg.linear.y = linear_y * vf
    pub.publish(msg)
    rate.sleep()

def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")

def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global wz
    global vf

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

    d= rospy.get_param("~distance_laser")
    vx= rospy.get_param("~forward_speed")
    wz= rospy.get_param("~rotation_speed")
    vf= rospy.get_param("~speed_factor")

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()