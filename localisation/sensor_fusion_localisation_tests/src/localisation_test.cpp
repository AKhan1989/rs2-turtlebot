#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("sensor_fusion_node") {
        // Define sensor QoS for topics like IMU, LIDAR, and camera
        rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                                    .durability(rclcpp::DurabilityPolicy::Volatile);

        // Use default QoS for odometry (usually reliable)
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SensorFusionNode::odom_callback, this, std::placeholders::_1)
        );

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", sensor_qos,
            std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1)
        );

        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", sensor_qos,
            std::bind(&SensorFusionNode::lidar_callback, this, std::placeholders::_1)
        );

        depth_camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", sensor_qos,
            std::bind(&SensorFusionNode::depth_camera_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node Started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        // TODO: Store/process odometry
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received IMU data");
        // TODO: Store/process IMU
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received lidar scan");
        // TODO: Use for obstacle detection / matching
    }

    void depth_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received depth image");
        // TODO: Could process pointclouds later
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_camera_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionNode>());
    rclcpp::shutdown();
    return 0;
}