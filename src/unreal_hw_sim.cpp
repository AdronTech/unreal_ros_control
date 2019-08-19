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

        if(new_socket < 0) break;

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
                // TODO: save Name
                // printf("\t\t\tName: %s", readbuffer);

                // read pos
                if (!read(readbuffer, 8))
                    break;

                // TODO: save pos
                uint64_t temp = ntohll(*(uint64_t *)readbuffer);
                double value = *(double *)&temp;
                // printf("\tPos: %f\n", value);
                pos = value;

                // read vel
                if (!read(readbuffer, 8))
                    break;

                // TODO: save vel

                // read eff
                if (!read(readbuffer, 8))
                    break;

                // TODO: save eff
            }
        }

        close(new_socket);
        printf("#### DISCONNECTED ####\n");
        connected = false;
    }
}

unreal_ros_control::UnrealHWSim::UnrealHWSim() : socketThread{}
{
    hardware_interface::JointStateHandle state_handle("joint1", &pos, &vel, &eff);
    jnt_state_interface.registerHandle(state_handle);
    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle pos_handle(state_handle, &cmd);
    jnt_pos_interface.registerHandle(pos_handle);
    registerInterface(&jnt_pos_interface);

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
    if(connected.load()){

        char *pointer = sendbuffer;

        *(uint16_t *)pointer = htons(2);
        pointer += 2;

        char text[100] = "joint1";

        *(uint16_t *)pointer = htons(strlen(text) + 1);
        pointer += 2;

        strcpy(pointer, text);
        pointer += strlen(text) + 1;


        uint64_t temp = htonll(*(uint64_t *)&cmd);
        *(uint64_t *)pointer = temp;
        pointer += 8;

        // neolib::hex_dump(sendbuffer, pointer - sendbuffer, std::cout);

        send(new_socket, sendbuffer, pointer - sendbuffer, 0);

        printf("cmd: %f\n", cmd);    
    }
}