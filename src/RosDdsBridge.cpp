#include <ros/ros.h>
#include "BridgeManager.hpp"
#include "ros_to_dds/RosToDdsPoseBridge.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_dds_bridge");
    ros::NodeHandle nh;

    BridgeManager mgr;

    auto pose_bridge = std::make_shared<ros_to_dds::RosToDdsPoseBridge>(nh);
    mgr.register_ros_to_dds_bridge(pose_bridge);

    if (!mgr.init_all())
    {
        ROS_ERROR("Failed to initialize one or more translators");
        return 1;
    }

    ROS_INFO("Bridge initialized. Spinning...");
    ros::spin();
    return 0;
}
