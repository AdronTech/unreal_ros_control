
#pragma once

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#if __BIG_ENDIAN__
# define htonll(x) (x)
# define ntohll(x) (x)
#else
# define htonll(x) (((uint64_t)htonl((x) & 0xFFFFFFFF) << 32) | htonl((x) >> 32))
# define ntohll(x) (((uint64_t)ntohl((x) & 0xFFFFFFFF) << 32) | ntohl((x) >> 32))
#endif

#define PORT 8080

namespace unreal_ros_control
{

class UnrealHWSim : public hardware_interface::RobotHW
{
private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    double cmd;
    double pos;
    double vel;
    double eff;

    int server_fd, new_socket, valread;
    char buffer[1024] = {0};

public:
    UnrealHWSim();

    void readSim();

    void writeSim();
};

} // namespace unreal_ros_control