#ifndef TWOFG_XMLRPC_HPP
#define TWOFG_XMLRPC_HPP

#include <string>
#include <xmlrpc-c/client_simple.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

#define TWOFG_ID 0xC0

class twofg_xmlrpc {
public:

    explicit twofg_xmlrpc(const std::string& ip_address);

    bool is_port_open();

    bool is_device_connected(int t_index, int id);
    bool is_busy(int t_index);
    bool is_gripped(int t_index);
    double get_external_width(int t_index);
    double get_internal_width(int t_index);
    int get_force(int t_index);
    double get_finger_length(int t_index);
    double get_finger_height(int t_index);
    double get_fingertip_offset(int t_index);
    bool get_finger_orientation_outward(int t_index);
    std::vector<double> get_width_values(int t_index, bool grip_mode);

    bool set_finger_length(int t_index, double t_length);
    bool set_finger_height(int t_index, double t_height);
    bool set_fingertip_offset(int t_index, double t_offset);
    bool set_finger_orientation(int t_index, bool t_orient);

    int grip_internal(int t_index, double t_width, int n_force, int p_speed, bool wait);
    int grip_external(int t_index, double t_width, int n_force, int p_speed, bool wait);

private:
    std::string server_url_;
    std::string server_ip;
    xmlrpc_c::clientSimple client_;   

};

#endif // TWOFG_XMLRPC_HPP
