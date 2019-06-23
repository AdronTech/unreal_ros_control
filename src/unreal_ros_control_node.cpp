#include "ros/ros.h"

#include "controller_manager/controller_manager.h"

#include <string>
#include "unreal_hw_sim.h"

unreal_ros_control::UnrealHWSim HWInterface;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unreal_ros_control_node");
    ros::NodeHandle nh;

    // std::string urdf_robot_description = "";
    // if(!nh.getParam("robot_description", urdf_robot_description))
    // {
    //     ROS_ERROR("FAILED!");
    // }
    // ROS_INFO("%s\n", urdf_robot_description.c_str());

    controller_manager::ControllerManager cm(&HWInterface, nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(2);
    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();

        HWInterface.readSim();
        cm.update(ts, d);
        HWInterface.writeSim();

        ROS_INFO("Test");

        // ros::spinOnce();
        rate.sleep();
    }

    spinner.stop();

    return 0;
}