#pragma once

#include "ros_to_dds/RosToDdsBridge.hpp"

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "../types/typesPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;

namespace ros_to_dds
{
class RosToDdsPoseBridge : public RosToDdsBridge
{
public:
    RosToDdsPoseBridge(ros::NodeHandle& nh);
    virtual ~RosToDdsPoseBridge();

    std::string ros_topic() const override { return "amcl_pose"; }
    std::string dds_topic() const override { return "amcl_pose"; }

    bool init() override;

    void set_dds_context(eprosima::fastdds::dds::DomainParticipant* participant,
                         eprosima::fastdds::dds::Publisher* publisher,
                         const std::string& robot_id) override;

private:
    void rosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::NodeHandle& nh_;
    ros::Subscriber ros_sub_;

    // DDS
    DomainParticipant* participant_;
    Publisher* publisher_;
    Topic* topic_;
    DataWriter* writer_;
    TypeSupport type_;

    // robot id loaded from environment (ROBOT_ID) with a default
    std::string robot_id_;

    // flags indicating whether participant/publisher are owned by this bridge
    bool own_participant_ = false;
    bool own_publisher_ = false;
};

} // namespace ros_to_dds
