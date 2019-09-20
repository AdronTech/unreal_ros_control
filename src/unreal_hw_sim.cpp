#include "unreal_hw_sim.h"
#include <chrono>
#include "neolib/hexdump.hpp"

bool unreal_ros_control::UnrealHWSim::read(char *buffer, uint32_t length)
{
    int offset = 0;
    while (offset < length)
    {
        valread = ::read(new_socket, buffer + offset, length - offset);
        if (valread <= 0)
            return false;
        offset += valread;
    }

    return true;
}

void unreal_ros_control::UnrealHWSim::socket_function()
{

    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    uint16_t nrJoints;
    uint16_t nameLength;

    // Creating socket file descriptor
    if ((server_fd = ::socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        ::perror("socket failed");
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = ::htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (::bind(server_fd, (struct sockaddr *)&address,
               sizeof(address)) < 0)
    {
        ::perror("bind failed");
    }

    if (::listen(server_fd, 3) < 0)
    {
        ::perror("listen");
    }
    ROS_INFO("Listen");

    while (running.load())
    {
        if ((new_socket = ::accept(server_fd, (struct sockaddr *)&address,
                                   (socklen_t *)&addrlen)) < 0)
        {
            ::perror("accept");
        }

        if (new_socket < 0)
            break;

        printf("#### ACCEPT ####\n");

        connected = true;

        while (running.load())
        {
            if (!read(readbuffer, 2))
                break;

            nrJoints = ntohs(*(uint16_t *)&readbuffer);

            for (uint16_t i = 0; i < nrJoints; i++)
            {
                // read name length
                if (!read(readbuffer, 2))
                    break;
                nameLength = ntohs(*(uint16_t *)&readbuffer);

                // read name
                if (!read(readbuffer, nameLength))
                    break;
                
                // save Name
                std::string label((char *)readbuffer);

                // read pos
                if (!read(readbuffer, 8))
                    break;

                // save pos
                uint64_t temp = ntohll(*(uint64_t *)readbuffer);
                double pos = *(double *)&temp;
                joint_information_[label].position = pos;

                // read vel
                if (!read(readbuffer, 8))
                    break;

                // save vel
                temp = ntohll(*(uint64_t *)readbuffer);
                double vel = *(double *)&temp;
                joint_information_[label].velocity = vel;


                // read eff
                if (!read(readbuffer, 8))
                    break;

                // save eff
                temp = ntohll(*(uint64_t *)readbuffer);
                double eff = *(double *)&temp;
                joint_information_[label].effort = eff;

            }
        }

        close(new_socket);
        printf("#### DISCONNECTED ####\n");
        connected = false;
    }
}

unreal_ros_control::UnrealHWSim::UnrealHWSim(std::vector<transmission_interface::TransmissionInfo> transmissions) : socketThread{}
{
    for (unsigned int j = 0; j < transmissions.size(); j++)
    {
        JointInfo joint_info;
        std::string label = transmissions[j].joints_[0].name_;
        joint_information_[label] = joint_info;

        printf("cmd: %f\n", cmd);
        std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
        const std::string &hardware_interface = joint_interfaces.front();

        hardware_interface::JointStateHandle state_handle(label, &joint_information_[label].position, &joint_information_[label].velocity, &joint_information_[label].effort);
        js_interface_.registerHandle(state_handle);
        registerInterface(&js_interface_);

        hardware_interface::     joint_handle(state_handle, &joint_information_[label].command);

        if (hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
        {
            pj_interface_.registerHandle(joint_handle);
        }
        else if (hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")
        {
            vj_interface_.registerHandle(joint_handle);
        }
        else
        {
            // ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "No matching hardware interface found for '"
            //                                                    << hardware_interface << "' while loading interfaces for " << joint_names_[j]);
            // return;
        }
    }

    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&vj_interface_);

    socketThread = std::thread{&unreal_ros_control::UnrealHWSim::socket_function, this};
}

void unreal_ros_control::UnrealHWSim::Destroy()
{
    running = false;
    if (::shutdown(server_fd, SHUT_RDWR) < 0)
    {
        ::fprintf(stderr, "shutdown: failed! %s\n", strerror(errno));
    }
    socketThread.join();
}

void unreal_ros_control::UnrealHWSim::readSim()
{
}

void unreal_ros_control::UnrealHWSim::writeSim()
{
    if (connected.load())
    {

        char *pointer = sendbuffer;
        *(uint16_t *)pointer = htons(joint_information_.size());
        pointer += 2;

        for (auto it = joint_information_.begin(); it != joint_information_.end(); ++it)
        {
            uint16_t label_length = it->first.length() + 1;

            *(uint16_t *)pointer = htons(label_length);
            pointer += 2;

            strcpy(pointer, it->first.c_str());
            pointer += label_length;

            uint64_t temp = htonll(*(uint64_t *)&it->second.command);
            *(uint64_t *)pointer = temp;
            pointer += 8;
        }
        // neolib::hex_dump(sendbuffer, pointer - sendbuffer, std::cout);

        send(new_socket, sendbuffer, pointer - sendbuffer, 0);
    }
}