#include "rclcpp/rclcpp.hpp"

class SensorFusionLocalisationNode : public rclcpp::Node {
public:
    SensorFusionLocalisationNode() : Node("sensor_fusion_localisation_node") {
        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Localisation Node Started");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionLocalisationNode>());
    rclcpp::shutdown();
    return 0;
}
