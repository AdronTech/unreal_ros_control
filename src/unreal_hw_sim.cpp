#include "unreal_hw_sim.h"

unreal_ros_control::UnrealHWSim::UnrealHWSim(){
    hardware_interface::JointStateHandle state_handle("joint1", &pos, &vel, &eff);
    jnt_state_interface.registerHandle(state_handle);
    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle vel_handle(state_handle, &cmd);
    jnt_vel_interface.registerHandle(vel_handle);
    registerInterface(&jnt_vel_interface);

    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
       
    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) 
    { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    } 
}

void unreal_ros_control::UnrealHWSim::readSim(){
    valread = ::read( new_socket , buffer, 1024); 
     
    uint64_t temp = ntohll(*(uint64_t *)buffer);

    double value = *(double *)&temp;
    printf("%f\n", value);
}

void unreal_ros_control::UnrealHWSim::writeSim(){
    cmd += 0.1;

    uint64_t tmp = htonll(*(uint64_t *)&cmd);
    *(uint64_t *)buffer = tmp;

    ::send(new_socket , buffer , 8, 0 ); 
}