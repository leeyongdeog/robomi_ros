#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply, quaternion_about_axis
class OdomPredictorNode:
    def __init__(self):
        rospy.init_node('odom_predictor_node')
        self.seq = 0
        self.have_odom = False
        self.have_bias = False
        self.max_imu_queue_length = rospy.get_param("~max_imu_queue_length", 1000)

        self.imu_queue = []
        self.imu_linear_acceleration_bias = Vector3(0.0, 0.0, 0.0)
        self.imu_angular_velocity_bias = Vector3(0.0, 0.0, 0.0)

        self.transform = dict()
        self.pose_covariance = []
        self.linear_velocity = []
        self.angular_velocity = []
        self.twist_covariance = []
        self.frame_id = 'odom'
        self.child_frame_id = 'base_footprint'

        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback, queue_size=100)
        self.imu_bias_sub = rospy.Subscriber("imu_bias", Imu, self.imu_bias_callback, queue_size=100)
        self.odometry_sub = rospy.Subscriber("odom", Odometry, self.odometry_callback, queue_size=100)

        self.odom_pub = rospy.Publisher("predicted_odom", Odometry, queue_size=100)
        self.transform_broadcaster = TransformBroadcaster()

        self.estimate_timestamp = rospy.Time.now()

    def odometry_callback(self, msg):
        if not self.have_bias:
            return

        # Clear old IMU measurements
        current_time = rospy.Time.now()
        imu_queue_temp = []
        for imu_msg in self.imu_queue:
            if imu_msg.header.stamp < msg.header.stamp:
                imu_queue_temp.append(imu_msg)

        self.imu_queue = imu_queue_temp

        # Extract useful information from the message
        self.transform = self.pose_msg_to_kindr(msg.pose.pose)
        self.pose_covariance = msg.pose.covariance
        self.linear_velocity = self.vector_msg_to_kindr(msg.twist.twist.linear)
        self.angular_velocity = self.vector_msg_to_kindr(msg.twist.twist.angular)
        self.twist_covariance = msg.twist.covariance

        # Reintegrate IMU messages
        try:
            for imu_msg in self.imu_queue:
                self.integrate_imu_data(imu_msg)
        except Exception as e:
            rospy.logerr("IMU integration failed, resetting everything: {}".format(e))
            self.have_bias = False
            self.have_odom = False
            self.imu_queue = []
            return

        self.have_odom = True

        # Publish the predicted odometry
        self.publish_odometry(current_time)

    def imu_callback(self, msg):
        if not self.have_bias or not self.have_odom:
            return

        # Clear old IMU measurements
        current_time = rospy.Time.now()
        if self.imu_queue and msg.header.stamp < self.imu_queue[-1].header.stamp:
            rospy.logerr("Latest IMU message occurred at time: {}. "
                          "This is before the previously received IMU message that occurred at: {}. "
                          "The current imu queue will be reset.".format(msg.header.stamp,
                                                                         self.imu_queue[-1].header.stamp))
            self.imu_queue = []

        # Remove old IMU messages if the queue size exceeds the maximum
        if len(self.imu_queue) > self.max_imu_queue_length:
            rospy.logwarn_throttle(10, "There have been over {} IMU messages since the last odometry update. "
                                       "The oldest measurement will be forgotten. "
                                       "This message is printed once every 10 seconds".format(self.max_imu_queue_length))
            self.imu_queue.pop(0)

        self.imu_queue.append(msg)

        try:
            self.integrate_imu_data(msg)
        except Exception as e:
            rospy.logerr("IMU integration failed, resetting everything: {}".format(e))
            self.have_bias = False
            self.have_odom = False
            self.imu_queue = []
            return

        self.publish_odometry(current_time)
        self.publish_tf(current_time)

    def imu_bias_callback(self, msg):
        self.imu_linear_acceleration_bias = self.vector_msg_to_kindr(msg.linear_acceleration)
        self.imu_angular_velocity_bias = self.vector_msg_to_kindr(msg.angular_velocity)

        self.have_bias = True

    def integrate_imu_data(self, imu_msg):
        delta_time = (imu_msg.header.stamp - self.estimate_timestamp).to_sec()
        gravity = Vector3(0.0, 0.0, -9.81)

        imu_linear_acceleration = self.vector_msg_to_kindr(imu_msg.linear_acceleration)
        imu_angular_velocity = self.vector_msg_to_kindr(imu_msg.angular_velocity)

        final_angular_velocity = (imu_angular_velocity - self.imu_angular_velocity_bias)
        delta_angle = delta_time * (final_angular_velocity + self.angular_velocity) / 2.0
        self.angular_velocity = final_angular_velocity

        # Calculate half delta rotation using tf functions
        half_delta_rotation = quaternion_about_axis(delta_angle[2] / 2.0, (0, 0, 1))
        self.transform.rotation = quaternion_multiply(self.transform.rotation, half_delta_rotation)

        delta_linear_velocity = delta_time * (imu_linear_acceleration +
                                              self.transform.rotation.inverse().rotate(gravity) -
                                              self.imu_linear_acceleration_bias)
        self.transform.position = self.transform.position + self.transform.rotation.rotate(delta_time * (self.linear_velocity + delta_linear_velocity / 2.0))
        self.linear_velocity += delta_linear_velocity

        # Apply the other half of the rotation using tf functions
        self.transform.rotation = quaternion_multiply(self.transform.rotation, half_delta_rotation)

        self.estimate_timestamp = imu_msg.header.stamp

    def pose_msg_to_kindr(self, pose_msg):
        transform = dict()
        transform['position'] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        transform['orientation'] = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        return transform

    def vector_msg_to_kindr(self, vector_msg):
        vector = [vector_msg.x, vector_msg.y, vector_msg.z]
        return vector

    def publish_odometry(self, current_time):
        if not self.have_odom:
            return

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.frame_id

        odom.pose.pose.position = Point(*self.transform['position'])
        odom.pose.pose.orientation = Quaternion(*self.transform['orientation'])

        odom.pose.covariance = self.pose_covariance

        odom.child_frame_id = self.child_frame_id
        odom.twist.twist.linear = Vector3(*self.linear_velocity)
        odom.twist.twist.angular = Vector3(0.0, 0.0, self.angular_velocity[2])  # Assuming 2D

        self.odom_pub.publish(odom)

    def publish_tf(self, current_time):
        if not self.have_odom:
            return
        self.transform_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = self.transform['position'][0]
        tf_msg.transform.translation.y = self.transform['position'][1]
        tf_msg.transform.translation.z = self.transform['position'][2]
        tf_msg.transform.rotation.x = self.transform['orientation'][0]
        tf_msg.transform.rotation.y = self.transform['orientation'][1]
        tf_msg.transform.rotation.z = self.transform['orientation'][2]
        tf_msg.transform.rotation.w = self.transform['orientation'][3]

        self.transform_broadcaster.sendTransform(tf_msg)

if __name__ == "__main__":
    try:
        node = OdomPredictorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
