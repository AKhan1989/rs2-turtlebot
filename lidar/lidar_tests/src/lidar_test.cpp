#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/qos.hpp"   // added this to modify subscriber to ensure we can listen to topic - atm it's not compatible

class LiDARTestNode : public rclcpp::Node {
public:
    LiDARTestNode() : Node("lidar_test_node") {
        // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "/scan", 10, std::bind(&LiDARTestNode::lidar_callback, this, std::placeholders::_1));

        // rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                                    .durability(rclcpp::DurabilityPolicy::Volatile);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile, std::bind(&LiDARTestNode::lidar_callback, this, std::placeholders::_1));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // int front_index = msg->ranges.size() / 2;  // Middle index represents the front (0°)
        int front_index = 0;    // Hard code it as 0 or 360 for the front lidar reading
        float lidar_distance = msg->ranges[front_index] - 0.150;    // Calibrate it 150mm is camera sensor to the front of the turtlebot
        
        // float middle_angle = msg->angle_min + (msg->angle_max - msg->angle_min) / 2;
        // int front_index = static_cast<int>((middle_angle - msg->angle_min) / msg->angle_increment);
        // float lidar_distance = msg->ranges[front_index];

        RCLCPP_INFO(this->get_logger(), "Front LiDAR Distance: %.3f m", lidar_distance);
        RCLCPP_INFO(this->get_logger(), "Angle Min: %.3f, Angle Max: %.3f, Number of readings: %zu",
                    msg->angle_min, msg->angle_max, msg->ranges.size());
        // Atm, it is starting at 0, scans up to 360 deg, and has 360 distance measurements.

        // Expected test distances
        std::vector<float> expected_values = {0.5, 0.25, 0.1, 0.05};
        float tolerance = 0.02;  // 2cm tolerance

        for (float expected : expected_values) {
            if (std::abs(lidar_distance - expected) <= tolerance) {
                RCLCPP_INFO(this->get_logger(), "Expected: %.3f m | PASS", expected);
            } else {
                RCLCPP_WARN(this->get_logger(), "Expected: %.3f m | FAIL", expected);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDARTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
