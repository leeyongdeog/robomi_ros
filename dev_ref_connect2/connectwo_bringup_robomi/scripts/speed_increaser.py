#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def limit_angular_speed(angular_speed):
    """회전 속도를 -3.3에서 3.3 사이로 제한"""
    if angular_speed > 3.3:
        return 3.3
    elif angular_speed < -3.3:
        return -3.3
    return angular_speed

def cmd_vel_callback(data):
    linear_x = data.linear.x
    angular_z = data.angular.z

    # 정지 상태에서 회전 속도를 3.5배로 증폭
    if linear_x == 0.0:
        if 0 < angular_z <= 2.5:
            data.angular.z = 3.5
        elif -2.5 <= angular_z < 0:
            data.angular.z = -3.5

    # 천천히 전진 또는 후진할 때 속도 증폭
    elif (-0.05 < linear_x < 0) or (0 < linear_x < 0.05):
        data.linear.x *= 3.0
        if 0 < angular_z <= 2.5:
            data.angular.z = 2.8
        elif -2.5 <= angular_z < 0:
            data.angular.z = -2.8
        data.angular.z = limit_angular_speed(data.angular.z)

    # 그 외의 경우
    else:
        data.linear.x *= 1.1
        data.angular.z *= 3.3
        data.angular.z = round(data.angular.z, 2)
        data.angular.z = limit_angular_speed(data.angular.z)

    # 증폭된 메시지를 다시 cmd_vel 토픽으로 게시
    cmd_vel_publisher.publish(data)

if __name__ == '__main__':
    rospy.init_node('speed_increaser', anonymous=True)

    # /cmd_vel 토픽에서 메시지를 받는 subscriber 생성
    rospy.Subscriber('speed_increase', Twist, cmd_vel_callback)

    # /cmd_vel 토픽으로 메시지를 게시하는 publisher 생성
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # 노드를 실행
    rospy.spin()
