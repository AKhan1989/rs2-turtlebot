#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/qos.hpp"   // added this to modify subscriber to ensure we can listen to topic - atm it's not compatible

class DepthCameraTest : public rclcpp::Node {
public:
    DepthCameraTest() : Node("depth_camera_test") {
        // subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/camera/depth/color/points", 10,
        //     std::bind(&DepthCameraTest::pointcloud_callback, this, std::placeholders::_1));
        // Define QoS profile compatible with depth camera publisher
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                                    .durability(rclcpp::DurabilityPolicy::Volatile);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", qos_profile,
            std::bind(&DepthCameraTest::pointcloud_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        int width = msg->width;
        int height = msg->height;
        
        int centre_x = width / 2;
        int centre_y = height / 2;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        int index = centre_y * width + centre_x;
        float depth = iter_z[index];  // Extract Z (depth) value

        RCLCPP_INFO(this->get_logger(), "Depth at center pixel: %.3f m", depth);

        // Expected test distances
        std::vector<float> expected_values = {0.5, 0.25, 0.1, 0.05};
        float tolerance = 0.05;  // 5cm tolerance

        for (float expected : expected_values) {
            if (std::abs(depth - expected) <= tolerance) {
                RCLCPP_INFO(this->get_logger(), "Expected: %.3f m | PASS", expected);
            } else {
                RCLCPP_WARN(this->get_logger(), "Expected: %.3f m | FAIL", expected);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthCameraTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
