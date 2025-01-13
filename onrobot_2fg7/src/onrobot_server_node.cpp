#include "rclcpp/rclcpp.hpp"
#include "onrobot_server.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OnrobotServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
