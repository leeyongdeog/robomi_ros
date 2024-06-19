#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"

class ImuOdometryNode {
public:
    ImuOdometryNode() : nh_private_("~"), have_bias_(false), have_odom_(false) {
        imu_sub_ = nh_.subscribe("imu", 100, &ImuOdometryNode::imuCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("fake_odom", 50);

        // Load parameters
        nh_private_.param("linear_acceleration_bias_x", linear_acceleration_bias_x_, 0.0);
        nh_private_.param("angular_velocity_bias_z", angular_velocity_bias_z_, 0.0);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        if (!have_bias_) {
            return;
        }

        // Extract IMU data
        double linear_acceleration_x = msg->linear_acceleration.x - linear_acceleration_bias_x_;
        double angular_velocity_z = msg->angular_velocity.z - angular_velocity_bias_z_;

        // TODO: Integrate IMU data to predict odometry using odom_predictor functions

        // For now, we'll publish a simple odometry message with IMU data
        nav_msgs::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x += linear_acceleration_x * 0.1;  // Example: Simple integration
        odom.twist.twist.angular.z += angular_velocity_z * 0.1;  // Example: Simple integration

        odom_pub_.publish(odom);

        have_odom_ = true;
    }

    void imuBiasCallback(const sensor_msgs::ImuConstPtr& msg) {
        linear_acceleration_bias_x_ = msg->linear_acceleration.x;
        angular_velocity_bias_z_ = msg->angular_velocity.z;
        have_bias_ = true;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;

    bool have_bias_;
    bool have_odom_;

    double linear_acceleration_bias_x_;
    double angular_velocity_bias_z_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_odometry_node");
    ImuOdometryNode imu_odometry_node;
    ros::spin();
    return 0;
}