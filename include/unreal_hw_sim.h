
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
#include <thread>

#if __BIG_ENDIAN__
#define htonll(x) (x)
#define ntohll(x) (x)
#else
#define htonll(x) (((uint64_t)htonl((x)&0xFFFFFFFF) << 32) | htonl((x) >> 32))
#define ntohll(x) (((uint64_t)ntohl((x)&0xFFFFFFFF) << 32) | ntohl((x) >> 32))
#endif

#define PORT 8080

namespace unreal_ros_control
{

class UnrealHWSim : public hardware_interface::RobotHW
{
private:
    std::atomic<bool> running{true};
    std::atomic<bool> connected{false};

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;

    double cmd;
    double pos;
    double vel;
    double eff;

    int server_fd, new_socket, valread;
    char buffer[1024] = {0};

    std::thread socketThread;

    void socket_function();

    bool read(char *buffer, uint32_t length);

public:
    UnrealHWSim();
    void Destroy();

    void readSim();

    void writeSim();
};

} // namespace unreal_ros_control