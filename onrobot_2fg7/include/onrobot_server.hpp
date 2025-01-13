#ifndef ONROBOT_SERVER_HPP
#define ONROBOT_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "twofg_xmlrpc.hpp"

#include "onrobot_msgs/srv/get_ext_width.hpp"
#include "onrobot_msgs/srv/get_int_width.hpp"
#include "onrobot_msgs/srv/get_finger_values.hpp"
#include "onrobot_msgs/srv/get_max_min_width.hpp"
#include "onrobot_msgs/srv/grip_external.hpp"
#include "onrobot_msgs/srv/grip_internal.hpp"
#include "onrobot_msgs/srv/is_conn.hpp"
#include "onrobot_msgs/srv/is_gripped.hpp"

class OnrobotServer : public rclcpp::Node
{
public:
    OnrobotServer();

private:
    std::unique_ptr<twofg_xmlrpc> twofg_;

    void handle_is_conn(
        const std::shared_ptr<onrobot_msgs::srv::IsConn::Request> request,
        std::shared_ptr<onrobot_msgs::srv::IsConn::Response> response);

    void handle_is_gripped(
        const std::shared_ptr<onrobot_msgs::srv::IsGripped::Request> request,
        std::shared_ptr<onrobot_msgs::srv::IsGripped::Response> response);

    void handle_get_ext_width(
        const std::shared_ptr<onrobot_msgs::srv::GetExtWidth::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GetExtWidth::Response> response);

    void handle_get_int_width(
        const std::shared_ptr<onrobot_msgs::srv::GetIntWidth::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GetIntWidth::Response> response);

    void handle_get_finger_values(
        const std::shared_ptr<onrobot_msgs::srv::GetFingerValues::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GetFingerValues::Response> response);

    void handle_get_max_min_width(
        const std::shared_ptr<onrobot_msgs::srv::GetMaxMinWidth::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GetMaxMinWidth::Response> response);

    void handle_grip_external(
        const std::shared_ptr<onrobot_msgs::srv::GripExternal::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GripExternal::Response> response);

    void handle_grip_internal(
        const std::shared_ptr<onrobot_msgs::srv::GripInternal::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GripInternal::Response> response);

    // Service objects
    rclcpp::Service<onrobot_msgs::srv::IsConn>::SharedPtr is_conn_service_;
    rclcpp::Service<onrobot_msgs::srv::IsGripped>::SharedPtr is_gripped_service_;
    rclcpp::Service<onrobot_msgs::srv::GetExtWidth>::SharedPtr get_ext_width_service_;
    rclcpp::Service<onrobot_msgs::srv::GetIntWidth>::SharedPtr get_int_width_service_;
    rclcpp::Service<onrobot_msgs::srv::GetFingerValues>::SharedPtr get_finger_values_service_;
    rclcpp::Service<onrobot_msgs::srv::GetMaxMinWidth>::SharedPtr get_max_min_width_service_;
    rclcpp::Service<onrobot_msgs::srv::GripExternal>::SharedPtr grip_external_service_;
    rclcpp::Service<onrobot_msgs::srv::GripInternal>::SharedPtr grip_internal_service_;

};

#endif // ONROBOT_SERVER_HPP