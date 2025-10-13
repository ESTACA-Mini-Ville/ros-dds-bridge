#pragma once

#include <string>

// Minimal forward declarations to avoid heavy includes in headers
namespace eprosima {
namespace fastdds {
namespace dds {
class DomainParticipant;
class Publisher;
}
}
}

// Base interface for bridges that convert ROS -> DDS
namespace ros_to_dds
{
class RosToDdsBridge
{
public:
    virtual ~RosToDdsBridge() = default;

    // topic names
    virtual std::string ros_topic() const = 0;
    virtual std::string dds_topic() const = 0;

    // initialize the bridge (create DDS types/writers as needed)
    virtual bool init() = 0;

    // optional: BridgeManager will call this to provide a shared participant/publisher
    virtual void set_dds_context(eprosima::fastdds::dds::DomainParticipant* participant,
                                 eprosima::fastdds::dds::Publisher* publisher,
                                 const std::string& robot_id = std::string())
    {
        (void)participant; (void)publisher; (void)robot_id;
    }
};

} // namespace ros_to_dds
