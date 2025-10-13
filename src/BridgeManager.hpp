#pragma once

#include <vector>
#include <memory>

#include "ros_to_dds/RosToDdsBridge.hpp"
#include "dds_to_ros/DdsToRosBridge.hpp"

#include <fastdds/dds/domain/DomainParticipant.hpp>

using namespace eprosima::fastdds::dds;

class BridgeManager
{
public:
    BridgeManager();
    ~BridgeManager();

    // registration APIs for each bridge direction
    void register_ros_to_dds_bridge(std::shared_ptr<ros_to_dds::RosToDdsBridge> b);
    void register_dds_to_ros_bridge(std::shared_ptr<dds_to_ros::DdsToRosBridge> b);

    bool init_all();

    // create/own a single participant/publisher per BridgeManager instance
    DomainParticipant* participant() const { return participant_; }
    Publisher* publisher() const { return publisher_; }

private:
    std::vector<std::shared_ptr<ros_to_dds::RosToDdsBridge>> ros_to_dds_bridges_;
    std::vector<std::shared_ptr<dds_to_ros::DdsToRosBridge>> dds_to_ros_bridges_;
    // DDS context
    DomainParticipant* participant_ = nullptr;
    Publisher* publisher_ = nullptr;
};
