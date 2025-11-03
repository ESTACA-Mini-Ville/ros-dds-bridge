#pragma once

#include "dds_to_ros/DdsToRosBridge.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "../types/typesPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;

namespace dds_to_ros
{
class DdsToRosVelBridge : public DdsToRosBridge
{
public:
    DdsToRosVelBridge(ros::NodeHandle& nh);
    virtual ~DdsToRosVelBridge();

    std::string ros_topic() const override { return "cmd_vel"; }
    std::string dds_topic() const override { return "cmd_vel"; }

    bool init() override;

    void set_dds_context(eprosima::fastdds::dds::DomainParticipant* participant,
                         eprosima::fastdds::dds::Subscriber* subscriber,
                         const std::string& robot_id = std::string()) override;

private:
    // internal listener that forwards DDS samples to ROS when robot_id matches
    class ReaderListener : public DataReaderListener
    {
    public:
        explicit ReaderListener(DdsToRosVelBridge* parent) : parent_(parent) {}
        void on_data_available(DataReader* reader) override;
    private:
        DdsToRosVelBridge* parent_;
    };

    ros::NodeHandle& nh_;
    ros::Publisher ros_pub_;

    // DDS objects
    DomainParticipant* participant_;
    Subscriber* subscriber_;
    Topic* topic_;
    DataReader* reader_;
    TypeSupport type_;

    // robot id loaded from BridgeManager (ROBOT_ID)
    std::string robot_id_;

    // ownership flags
    bool own_participant_ = false;
    bool own_subscriber_ = false;

    ReaderListener listener_;
};

} // namespace dds_to_ros
