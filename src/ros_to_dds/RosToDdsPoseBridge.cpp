#include "ros_to_dds/RosToDdsPoseBridge.hpp"

#include <cstdlib>
#include <iostream>

using namespace eprosima::fastdds::dds;

namespace ros_to_dds
{
RosToDdsPoseBridge::RosToDdsPoseBridge(ros::NodeHandle& nh)
    : nh_(nh), participant_(nullptr), publisher_(nullptr), writer_(nullptr)
{
}

void RosToDdsPoseBridge::set_dds_context(eprosima::fastdds::dds::DomainParticipant* participant,
                                 eprosima::fastdds::dds::Publisher* publisher,
                                 const std::string& robot_id)
{
    participant_ = participant;
    publisher_ = publisher;
    own_participant_ = false;
    own_publisher_ = false;
    if (!robot_id.empty())
    {
        robot_id_ = robot_id;
    }
}

RosToDdsPoseBridge::~RosToDdsPoseBridge()
{
    if (writer_)
    {
        if (publisher_)
            publisher_->delete_datawriter(writer_);
    }
    if (publisher_)
    {
        if (own_publisher_ && participant_)
            participant_->delete_publisher(publisher_);
    }
    if (topic_ && participant_)
    {
        participant_->delete_topic(topic_);
    }
    if (participant_)
    {
        if (own_participant_)
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }
}

bool RosToDdsPoseBridge::init()
{
    if (!participant_)
    {
        if (robot_id_.empty())
        {
            const char* env_id = std::getenv("ROBOT_ID");
            if (env_id == nullptr || env_id[0] == '\0')
            {
                std::cerr << "FATAL: environment variable ROBOT_ID is not set or empty. Please set ROBOT_ID and restart." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            robot_id_ = env_id;
        }

        DomainParticipantQos qos;
        qos.name(robot_id_ + "_pose_bridge");

        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, qos);
        if (!participant_) return false;
        own_participant_ = true;
    }

    type_ = TypeSupport(new PoseWithCovarianceStampedPubSubType());
    type_.register_type(participant_);

    topic_ = participant_->create_topic(dds_topic().c_str(), "PoseWithCovarianceStamped", TOPIC_QOS_DEFAULT);
    if (!topic_) return false;

    if (!publisher_)
    {
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
        if (!publisher_) return false;
        own_publisher_ = true;
    }

    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, nullptr);
    if (!writer_) return false;

    ros_sub_ = nh_.subscribe(ros_topic(), 10, &RosToDdsPoseBridge::rosCallback, this);

    return true;
}

void RosToDdsPoseBridge::rosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (!writer_) return;

    std::cout << "RosToDdsPoseBridge: received ROS message, translating and publishing to DDS" << std::endl;

    PoseWithCovarianceStamped dds_msg;

    dds_msg.header().seq(msg->header.seq);
    dds_msg.header().stamp_secs(msg->header.stamp.sec);
    dds_msg.header().stamp_nsecs(msg->header.stamp.nsec);
    dds_msg.header().frame_id(msg->header.frame_id);

    dds_msg.pose().pose().position().x(msg->pose.pose.position.x);
    dds_msg.pose().pose().position().y(msg->pose.pose.position.y);
    dds_msg.pose().pose().position().z(msg->pose.pose.position.z);

    dds_msg.pose().pose().orientation().x(msg->pose.pose.orientation.x);
    dds_msg.pose().pose().orientation().y(msg->pose.pose.orientation.y);
    dds_msg.pose().pose().orientation().z(msg->pose.pose.orientation.z);
    dds_msg.pose().pose().orientation().w(msg->pose.pose.orientation.w);

    auto &cov = dds_msg.pose().covariance();
    for (unsigned int i = 0; i < 36; ++i)
    {
        cov[i] = msg->pose.covariance[i];
    }

    dds_msg.robot_id(robot_id_.c_str());

    writer_->write(&dds_msg);
}

} // namespace ros_to_dds
