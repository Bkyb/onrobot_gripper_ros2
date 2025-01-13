#include "onrobot_server.hpp"

OnrobotServer::OnrobotServer()
    : Node("onrobot_server")
{
    this->declare_parameter("ip", "");
    std::string server_url = this->get_parameter("ip").as_string();
    
    twofg_ = std::make_unique<twofg_xmlrpc>(server_url); 

    is_conn_service_ = this->create_service<onrobot_msgs::srv::IsConn>(
        "is_conn",
        std::bind(&OnrobotServer::handle_is_conn, this, std::placeholders::_1, std::placeholders::_2));

    is_gripped_service_ = this->create_service<onrobot_msgs::srv::IsGripped>(
        "is_gripped",
        std::bind(&OnrobotServer::handle_is_gripped, this, std::placeholders::_1, std::placeholders::_2));
    
    get_ext_width_service_ = this->create_service<onrobot_msgs::srv::GetExtWidth>(
        "get_ext_width",
        std::bind(&OnrobotServer::handle_get_ext_width, this, std::placeholders::_1, std::placeholders::_2));

    get_int_width_service_ = this->create_service<onrobot_msgs::srv::GetIntWidth>(
        "get_int_width",
        std::bind(&OnrobotServer::handle_get_int_width, this, std::placeholders::_1, std::placeholders::_2));

    get_finger_values_service_ = this->create_service<onrobot_msgs::srv::GetFingerValues>(
        "get_finger_values",
        std::bind(&OnrobotServer::handle_get_finger_values, this, std::placeholders::_1, std::placeholders::_2));

    get_max_min_width_service_ = this->create_service<onrobot_msgs::srv::GetMaxMinWidth>(
        "get_max_min_width",
        std::bind(&OnrobotServer::handle_get_max_min_width, this, std::placeholders::_1, std::placeholders::_2));

    grip_external_service_ = this->create_service<onrobot_msgs::srv::GripExternal>(
        "grip_external",
        std::bind(&OnrobotServer::handle_grip_external, this, std::placeholders::_1, std::placeholders::_2));

    grip_internal_service_ = this->create_service<onrobot_msgs::srv::GripInternal>(
        "grip_internal",
        std::bind(&OnrobotServer::handle_grip_internal, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "OnrobotServer is ready to handle requests.");
}


void OnrobotServer::handle_is_conn(
    const std::shared_ptr<onrobot_msgs::srv::IsConn::Request> request,
    std::shared_ptr<onrobot_msgs::srv::IsConn::Response> response)
{
    int index = request->index;
    int id = request->id;
    
    response->success = twofg_->is_device_connected(index,id);
    RCLCPP_INFO(this->get_logger(), "Device is  %s", response->success ? "connected" : "not connected");
}

void OnrobotServer::handle_is_gripped(
    const std::shared_ptr<onrobot_msgs::srv::IsGripped::Request> request,
    std::shared_ptr<onrobot_msgs::srv::IsGripped::Response> response)
{
    int index = request->index;

    response->success = twofg_->is_gripped(index); // Example value
    RCLCPP_INFO(this->get_logger(), "The gripper %s grip the object.", response->success ? "did" : "did not");
}


void OnrobotServer::handle_get_ext_width(
    const std::shared_ptr<onrobot_msgs::srv::GetExtWidth::Request> request,
    std::shared_ptr<onrobot_msgs::srv::GetExtWidth::Response> response)
{
    int index = request->index;
    response->width = twofg_->get_external_width(index);
    RCLCPP_INFO(this->get_logger(), "The external width is %.2f mm", response->width);
}

void OnrobotServer::handle_get_int_width(
    const std::shared_ptr<onrobot_msgs::srv::GetIntWidth::Request> request,
    std::shared_ptr<onrobot_msgs::srv::GetIntWidth::Response> response)
{
    int index = request->index;
    response->width = twofg_->get_internal_width(index);
    RCLCPP_INFO(this->get_logger(), "The internal width is %.2f mm", response->width);
}

void OnrobotServer::handle_get_finger_values(
        const std::shared_ptr<onrobot_msgs::srv::GetFingerValues::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GetFingerValues::Response> response)
{
    int index = request->index;
    response->length = twofg_->get_finger_length(index);
    if(response->length < 0) return;
    response->height = twofg_->get_finger_height(index);
    response->offset = twofg_->get_fingertip_offset(index);
    response->orient = twofg_->get_finger_orientation_outward(index);
    RCLCPP_INFO(this->get_logger(),
                "Current finger length is %.2f mm, finger height is %.2f mm, fingertip offset is %.2f mm and %s"
                ,response->length, response->height, response->offset, response->orient?"finger orientation is outwards":"finger orientation is inwards"
    );
}

void OnrobotServer::handle_get_max_min_width(
        const std::shared_ptr<onrobot_msgs::srv::GetMaxMinWidth::Request> request,
        std::shared_ptr<onrobot_msgs::srv::GetMaxMinWidth::Response> response)
{
    int index = request->index;
    bool grip = request->grip;
    std::vector<double> result;
    result = twofg_->get_width_values(index,grip);
    response->min_width = result[0];
    response->max_width = result[1];
    RCLCPP_INFO(this->get_logger(), "The minimum %s width is %.2f mm, maximum %s width is %.2f mm", 
                grip? "external":"internal",response->min_width, grip? "external":"internal", response->max_width);
}

void OnrobotServer::handle_grip_external(
    const std::shared_ptr<onrobot_msgs::srv::GripExternal::Request> request,
    std::shared_ptr<onrobot_msgs::srv::GripExternal::Response> response)
{
    int index = request->index;
    double width = request->width;
    int force = request->force;
    int speed = request->speed;
    bool wait = request->is_wait;
    response->success = twofg_->grip_external(index,width,force,speed,wait); 
    int status = response->success;

    if (status == 1) RCLCPP_INFO(this->get_logger(), "The gripper gripped the object.");
    else if (status == 0) RCLCPP_INFO(this->get_logger(), "The gripping operation is end.");
    else if (status == -1) RCLCPP_ERROR(this->get_logger(), "Error: Device is not connected.");
    else if (status == -2) RCLCPP_ERROR(this->get_logger(), "Error: Invalid input.");
    else if (status == -3) RCLCPP_ERROR(this->get_logger(), "Error: Device is busy.");
    else RCLCPP_ERROR(this->get_logger(), "Error");
    
}

void OnrobotServer::handle_grip_internal(
    const std::shared_ptr<onrobot_msgs::srv::GripInternal::Request> request,
    std::shared_ptr<onrobot_msgs::srv::GripInternal::Response> response)
{
    int index = request->index;
    double width = request->width;
    int force = request->force;
    int speed = request->speed;
    bool wait = request->is_wait;
    response->success = twofg_->grip_internal(index,width,force,speed,wait); 
    
    int status = response->success;
    if (status == 1) RCLCPP_INFO(this->get_logger(), "The gripper gripped the object.");
    else if (status == 0) RCLCPP_INFO(this->get_logger(), "The gripping operation is end.");
    else if (status == -1) RCLCPP_ERROR(this->get_logger(), "Error: Device is not connected.");
    else if (status == -2) RCLCPP_ERROR(this->get_logger(), "Error: Invalid input.");
    else if (status == -3) RCLCPP_ERROR(this->get_logger(), "Error: Device is busy.");
    else RCLCPP_ERROR(this->get_logger(), "Error");
    
}
