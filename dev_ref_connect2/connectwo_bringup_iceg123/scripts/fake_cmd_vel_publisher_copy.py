#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3 

rospy.init_node('cmd_vel_publisher')

cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=30) 


def cmd_vel_callback(data): #이동값을 cmd_vel에서 받아오는 함수
    global vx, vy, vz, angular_x, angular_y, angular_z
    vx = data.linear.x #이동값
    vy = data.linear.y
    vz = data.linear.z

    angular_x = data.angular.x #회전값
    angular_y = data.angular.y #회전값
    angular_z = data.angular.z #회전값


cmd_vel_sub = rospy.Subscriber("fake_cmd_vel", Twist, cmd_vel_callback) #cmd_vel값을 subscriber로 받아오는 구문
vx = 0.0  
vy = 0.0
vz = 0.0

angular_x = 0.0
angular_y = 0.0
angular_z = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

# r = rospy.Rate(144.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # since all odometry is 6DOF, create a quaternion from yaw
    # publish the transform over tf

    # publish the message
    cmd_vel = Twist()
    if vx == 0.0 and angular_z == 0.0:
        # If no message received, set cmd_vel to zero
        cmd_vel.linear = Vector3(0, 0, 0)
        cmd_vel.angular = Vector3(0, 0, 0)
    else:
    # set the velocity
        # angular_z = 2.0 * angular_z
        cmd_vel.linear = Vector3(vx, vy, vz)
        cmd_vel.angular = Vector3(angular_x, angular_y, angular_z)

    # publish the message
    cmd_vel_pub.publish(cmd_vel)

    last_time = current_time
    # r.sleep()