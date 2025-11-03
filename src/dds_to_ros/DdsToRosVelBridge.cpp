#include "dds_to_ros/DdsToRosVelBridge.hpp"

#include <iostream>
#include <cstdlib>
#include <string>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

using namespace eprosima::fastdds::dds;

namespace dds_to_ros
{

DdsToRosVelBridge::DdsToRosVelBridge(ros::NodeHandle& nh)
    : nh_(nh), participant_(nullptr), subscriber_(nullptr), topic_(nullptr), reader_(nullptr),
      type_(nullptr), own_participant_(false), own_subscriber_(false), listener_(this)
{
}

DdsToRosVelBridge::~DdsToRosVelBridge()
{
    if (reader_)
    {
        if (subscriber_)
            subscriber_->delete_datareader(reader_);
        reader_ = nullptr;
    }
    if (subscriber_)
    {
        if (own_subscriber_ && participant_)
            participant_->delete_subscriber(subscriber_);
        subscriber_ = nullptr;
    }
    if (topic_ && participant_)
    {
        participant_->delete_topic(topic_);
        topic_ = nullptr;
    }
    if (participant_)
    {
        if (own_participant_)
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
        participant_ = nullptr;
    }
}

void DdsToRosVelBridge::set_dds_context(eprosima::fastdds::dds::DomainParticipant* participant,
                                        eprosima::fastdds::dds::Subscriber* subscriber,
                                        const std::string& robot_id)
{
    participant_ = participant;
    subscriber_ = subscriber;
    own_participant_ = false;
    own_subscriber_ = false;
    if (!robot_id.empty())
    {
        robot_id_ = robot_id;
    }
}

bool DdsToRosVelBridge::init()
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
        qos.name(robot_id_ + "_vel_bridge");

        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, qos);
        if (!participant_) return false;
        own_participant_ = true;
    }

    // register the Twist type
    type_ = TypeSupport(new TwistPubSubType());
    type_.register_type(participant_);

    topic_ = participant_->create_topic(dds_topic().c_str(), "Twist", TOPIC_QOS_DEFAULT);
    if (!topic_) return false;

    if (!subscriber_)
    {
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
        if (!subscriber_) return false;
        own_subscriber_ = true;
    }

    reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &listener_);
    if (!reader_) return false;

    ros_pub_ = nh_.advertise<geometry_msgs::Twist>(ros_topic(), 10);

    return true;
}

// Implementation of the nested ReaderListener
void DdsToRosVelBridge::ReaderListener::on_data_available(DataReader* reader)
{
    if (!parent_) return;

    Twist dds_msg;
    SampleInfo info;

    // take all available samples and publish matched ones to ROS
    while (reader->take_next_sample(&dds_msg, &info) == eprosima::fastdds::dds::RETCODE_OK)
    {

        if (!info.valid_data) continue;

        try
        {
            std::string msg_robot_id = dds_msg.robot_id();      

            if (!parent_->robot_id_.empty() && msg_robot_id != parent_->robot_id_)
            {
                // not for this robot, ignore
                continue;
            }

            geometry_msgs::Twist ros_msg;
            // linear
            ros_msg.linear.x = dds_msg.linear().x();
            ros_msg.linear.y = dds_msg.linear().y();
            ros_msg.linear.z = dds_msg.linear().z();
            // angular
            ros_msg.angular.x = dds_msg.angular().x();
            ros_msg.angular.y = dds_msg.angular().y();
            ros_msg.angular.z = dds_msg.angular().z();

            parent_->ros_pub_.publish(ros_msg);

            std::cout << "DdsToRosVelBridge: published ROS cmd_vel message for robot_id='" << msg_robot_id << "'" << std::endl;

        }
        catch (const std::exception& e)
        {
            std::cerr << "DdsToRosVelBridge: exception while translating/publishing message: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "DdsToRosVelBridge: unknown exception while translating/publishing message" << std::endl;
        }
    }
}

} // namespace dds_to_ros
