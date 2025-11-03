#pragma once

// Base interface for bridges that convert DDS -> ROS
#include <string>
namespace eprosima {
namespace fastdds {
namespace dds {
class DomainParticipant;
class Subscriber;
}
}
}

namespace dds_to_ros
{
class DdsToRosBridge
{
public:
    virtual ~DdsToRosBridge() = default;

    virtual std::string ros_topic() const = 0;
    virtual std::string dds_topic() const = 0;

    virtual bool init() = 0;

    virtual void set_dds_context(eprosima::fastdds::dds::DomainParticipant* participant,
                                 eprosima::fastdds::dds::Subscriber* subscriber,
                                 const std::string& robot_id = std::string())
    {
        (void)participant; (void)subscriber; (void)robot_id;
    }
};

} // namespace dds_to_ros
