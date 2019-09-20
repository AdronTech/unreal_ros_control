#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include <transmission_interface/transmission_parser.h>

#include <string>
#include "unreal_hw_sim.h"

#include <signal.h>
#include <atomic>
#include <vector>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "unreal_ros_control_node");
    ros::NodeHandle nh;

    std::string urdf_robot_description = "";
    if (!nh.getParam("robot_description", urdf_robot_description))
    {
        ROS_ERROR("FAILED!");
    }
    // ROS_INFO("%s\n", urdf_robot_description.c_str());

    std::vector<transmission_interface::TransmissionInfo> transmissions_;
    transmission_interface::TransmissionParser::parse(urdf_robot_description, transmissions_);

    // for (int i = 0; i < transmissions_.size(); i++)
    // {
    //     ROS_INFO("%s\n", transmissions_[i].name_.c_str());
    // }
    
    unreal_ros_control::UnrealHWSim HWInterface(transmissions_);

    controller_manager::ControllerManager cm(&HWInterface, nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();

        HWInterface.readSim();
        cm.update(ts, d);
        HWInterface.writeSim();

        ros::spinOnce();
        rate.sleep();
    }
    HWInterface.Destroy();

    spinner.stop();

    return 0;
}