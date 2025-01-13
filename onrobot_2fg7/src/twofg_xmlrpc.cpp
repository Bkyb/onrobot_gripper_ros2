#include "twofg_xmlrpc.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>

twofg_xmlrpc::twofg_xmlrpc(const std::string& ip_address)
    : server_url_("http://" + ip_address + ":41414/") {server_ip = ip_address;}

// Check is port open
bool twofg_xmlrpc::is_port_open(){
    // initialize socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock<0) return false;

    // set server address
    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(41414);
    inet_pton(AF_INET, server_ip.c_str(), &serverAddr.sin_addr);

    // set socket nonblocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // try to connect
    int result = connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if (result < 0 && errno != EINPROGRESS) {close(sock); return false;}

    // timeout
    fd_set writeSet;
    FD_ZERO(&writeSet);
    FD_SET(sock, &writeSet);
    timeval timeout{};
    timeout.tv_sec = 5; // 5 sec
    timeout.tv_usec = 0;
    result = select(sock + 1, nullptr, &writeSet, nullptr, &timeout);
    if (result <= 0) {close(sock); return false;}

    // check connect success
    int optval;
    socklen_t optlen = sizeof(optval);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &optval, &optlen) < 0 || optval != 0) 
    {close(sock);return false;}

    close(sock);
    return true;
}

/**
 * @brief Check if the 2FG device is connected.
 * @param t_index The device position (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return [bool] True if connected, False otherwise.
 */
bool twofg_xmlrpc::is_device_connected(int t_index, int id) {
    if(!is_port_open()){
        std::cerr << "Warning: Failed to connect to ComputeBox due to a timeout." << std::endl;
        return false;
    }
    try {     
        xmlrpc_c::value result;
        client_.call(server_url_, "cb_is_device_connected", "ii", &result, t_index, id);
        bool connected = xmlrpc_c::value_boolean(result);
        if (!connected) {
            std::cerr << "Warning: Device not connected on the given instance." << std::endl;
        }
        return connected;
    } catch (const std::exception& e) {
        std::cerr << "Error in is_device_connected: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Check whether the gripper is busy.
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return True if the gripper busy, False otherwise.
 */
bool twofg_xmlrpc::is_busy(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return true;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_busy", "i", &result, t_index);
    return xmlrpc_c::value_boolean(result);
}

/**
 * @brief Checks if the gripper is gripping an object.
 * @param t_index The device position (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return True if gripping, False otherwise.
 */
bool twofg_xmlrpc::is_gripped(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return false;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_grip_detected", "i", &result, t_index);
    return xmlrpc_c::value_boolean(result);
}

/**
 * @brief Get the current external grip width.
 * @param t_index The device position (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return The external grip width in mm, -1.0 if device not connected
 */
double twofg_xmlrpc::get_external_width(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return -1.0;  
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_external_width", "i", &result, t_index);
    return xmlrpc_c::value_double(result);
}

/**
 * @brief Get the current internal grip width
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return The internal width in mm, -1.0 if device not connected
 */
double twofg_xmlrpc::get_internal_width(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return -1.0;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_internal_width", "i", &result, t_index);
    return xmlrpc_c::value_double(result);
}

/**
 * @brief Get the current force
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return Force in N, return empty if device didn't connected
 */
int twofg_xmlrpc::get_force(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return {};
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_force", "i", &result, t_index);
    return xmlrpc_c::value_int(result);
}

/**
 * @brief Get the current gripper finger width
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return The finger width in mm, -1.0 if device not connected 
 */
double twofg_xmlrpc::get_finger_length(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return -1.0;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_finger_length", "i", &result, t_index);
    return xmlrpc_c::value_double(result);
}

/**
 * @brief Get the current gripper finger height.
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return The finger height width in mm, -1.0 if device not connected 
 */
double twofg_xmlrpc::get_finger_height(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return -1.0;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_finger_height", "i", &result, t_index);
    return xmlrpc_c::value_double(result);
}


/**
 * @brief Get the current finger offset(a half of fingertip thickness)
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return finger offset in mm, -1.0 if device not connected 
 */
double twofg_xmlrpc::get_fingertip_offset(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return -1.0;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_fingertip_offset", "i", &result, t_index);
    return xmlrpc_c::value_double(result);
}

/**
 * @brief Get the current finger orientation
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @return True [┌ ┐] false[┐ ┌]
 */
bool twofg_xmlrpc::get_finger_orientation_outward(int t_index) {
    if (!is_device_connected(t_index, TWOFG_ID)) return false;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_get_finger_orientation_outward", "i", &result, t_index);
    return xmlrpc_c::value_boolean(result);
}

/**
 * @brief Get current minimum and maximum width
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @param grip_mode True : external, False : internal
 * @return Minimum width, maximum width
 */
std::vector<double> twofg_xmlrpc::get_width_values(int t_index, bool grip_mode) {
    if (!is_device_connected(t_index, TWOFG_ID)) return {};
    xmlrpc_c::value max_width;
    xmlrpc_c::value min_width;
    if(grip_mode){
        client_.call(server_url_, "twofg_get_max_external_width", "i", &max_width, t_index);
        client_.call(server_url_, "twofg_get_min_external_width", "i", &min_width, t_index);
    }else{
        client_.call(server_url_, "twofg_get_max_internal_width", "i", &max_width, t_index);
        client_.call(server_url_, "twofg_get_min_internal_width", "i", &min_width, t_index);
    }
    std::vector<double> result;
    result.push_back(xmlrpc_c::value_double(max_width));
    result.push_back(xmlrpc_c::value_double(min_width));

    return result;
}

/**
 * @brief Set current finger length(if you use another finger)
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).'
 * @param t_lenght Target finger length
 * @return True if the operation was successful, otherwise False
 */
bool twofg_xmlrpc::set_finger_length(int t_index, double t_length) {
    if (!is_device_connected(t_index, TWOFG_ID)) return false;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_set_finger_length", "id", &result, t_index, t_length);
    return xmlrpc_c::value_boolean(result);
}

/**
 * @brief Set current finger height(if you use another finger)
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).'
 * @param t_height Target finger height
* @return True if the operation was successful, otherwise False
 */
bool twofg_xmlrpc::set_finger_height(int t_index, double t_height) {
    if (!is_device_connected(t_index, TWOFG_ID)) return false;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_set_finger_height", "id", &result, t_index, t_height);
    return xmlrpc_c::value_boolean(result);
}

/**
 * @brief Set current fingertip offset(a half of fingertip thickness)
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).'
 * @param t_offset Target finger offset
 * @return True if the operation was successful, otherwise False
 */
bool twofg_xmlrpc::set_fingertip_offset(int t_index, double t_offset) {
    if (!is_device_connected(t_index, TWOFG_ID)) return false;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_set_fingertip_offset", "id", &result, t_index, t_offset);
    return xmlrpc_c::value_boolean(result);
}

/**
 * @brief Set current finger orientation
 * @param t_index The position of the device (0 for single, 1 for dual primary, 2 for dual secondary).
 * @param t_orient Target finger orientation True [┌ ┐] false[┐ ┌]
 * @return True if the operation was successful, otherwise False
 */
bool twofg_xmlrpc::set_finger_orientation(int t_index, bool t_orient) {
    if (!is_device_connected(t_index, TWOFG_ID)) return false;
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_set_finger_orientation", "ib", &result, t_index, t_orient);
    return xmlrpc_c::value_boolean(result);
}

/**
 * @brief Internal grip operation, moving the gripper to a specified position.
 * @param t_index The device position (0 for single, 1 for dual primary, 2 for dual secondary).
 * @param t_width The target grip width in millimeters (mm).
 * @param n_force The force applied during gripping, in newtons (N) (range: 20 to 140).
 * @param p_speed The speed of the operation as a percentage (range: 10 to 100).
 * @param wait Whether to wait for the operation to complete.
 * @return 1 if gripped, 0 if not gripped, -1 if the device is not connected, 
 *        -2 for invalid input, or -3 if the device is busy.
 */
int twofg_xmlrpc::grip_internal(int t_index, double t_width, int n_force, int p_speed, bool wait) {
    if (!is_device_connected(t_index, TWOFG_ID)) return -1;
    if (is_busy(t_index)) return -3;
    // get max, min width
    xmlrpc_c::value max_width, min_width;
    client_.call(server_url_, "twofg_get_max_internal_width","i",&max_width, t_index);
    double const max = xmlrpc_c::value_double(max_width);
    client_.call(server_url_, "twofg_get_min_internal_width","i",&min_width, t_index);
    double const min = xmlrpc_c::value_double(min_width);

    if(t_width > max || t_width < min){
        std::cerr<<"Invalid 2FG width parameter, "<<min<<"-"<<max<<" is valid only"<<std::endl;
        return -2;
    }
    if(n_force > 140 || n_force < 20){
        std::cerr<<"Invalid 2FG force parameter, 20-140 is valid only"<<std::endl;
        return -2;
    }
    if(p_speed > 100 || p_speed < 10){
        std::cerr<<"Invalid 2FG speed parameter, 10-100 is valid only"<<std::endl;
        return -2;
    }
    
    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_grip_internal", "idii", &result, t_index, t_width, n_force, p_speed);
    

    if(wait){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        bool check_busy = is_busy(t_index);
        auto start_time = std::chrono::steady_clock::now();
        while (check_busy)
        {
            check_busy = is_busy(t_index);
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);

            if(elapsed_time.count() >= 10) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 100ms            
        }
        if(is_gripped(t_index)) return 1;
       
    }

    // "wait is false" or "is_gripped is false"
    return xmlrpc_c::value_int(result);
}

/**
 * @brief External grip operation, moving the gripper to a specified position.
 * @param t_index The device position (0 for single, 1 for dual primary, 2 for dual secondary).
 * @param t_width The target grip width in millimeters (mm).
 * @param n_force The force applied during gripping, in newtons (N) (range: 20 to 140).
 * @param p_speed The speed of the operation as a percentage (range: 10 to 100).
 * @param wait Whether to wait for the operation to complete.
 * @return 1 if gripped, 0 if not gripped, -1 if the device is not connected, 
 *        -2 for invalid input, or -3 if the device is busy.
 */
int twofg_xmlrpc::grip_external(int t_index, double t_width, int n_force, int p_speed, bool wait) {
    if (!is_device_connected(t_index, TWOFG_ID)) return -1;
    if (is_busy(t_index)) return -3;
    // get max, min width
    xmlrpc_c::value max_width, min_width;
    client_.call(server_url_, "twofg_get_max_external_width","i",&max_width, t_index);
    double const max = xmlrpc_c::value_double(max_width);
    client_.call(server_url_, "twofg_get_min_external_width","i",&min_width, t_index);
    double const min = xmlrpc_c::value_double(min_width);

    if(t_width > max || t_width < min){
        std::cerr<<"Invalid 2FG width parameter, "<<min<<"-"<<max <<" is valid only."<<std::endl;
        return -2;
    }
    if(n_force > 140 || n_force < 20){
        std::cerr<<"Invalid 2FG force parameter, 20-140 is valid only."<<std::endl;
        return -2;
    }
    if(p_speed > 100 || p_speed < 10){
        std::cerr<<"Invalid 2FG speed parameter, 10-100 is valid only."<<std::endl;
        return -2;
    }

    xmlrpc_c::value result;
    client_.call(server_url_, "twofg_grip_external", "idii", &result, t_index, t_width, n_force, p_speed);

    
    if(wait){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        bool check_busy = is_busy(t_index);
        auto start_time = std::chrono::steady_clock::now();
        while (check_busy)
        {
            check_busy = is_busy(t_index);
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);

            if(elapsed_time.count() >= 10) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 100ms            
        }
        if(is_gripped(t_index)) return 1;
    }

    // if wait is false, or if is_gripped is false
    return xmlrpc_c::value_int(result);
}
