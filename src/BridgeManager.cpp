#include "BridgeManager.hpp"
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <iostream>
#include <cstdlib>
#include <string>

BridgeManager::BridgeManager() = default;
BridgeManager::~BridgeManager()
{
    if (participant_)
    {
        if (publisher_)
        {
            participant_->delete_publisher(publisher_);
            publisher_ = nullptr;
        }

        DomainParticipantFactory::get_instance()->delete_participant(participant_);
        participant_ = nullptr;
    }
}

void BridgeManager::register_ros_to_dds_bridge(std::shared_ptr<ros_to_dds::RosToDdsBridge> b)
{
    ros_to_dds_bridges_.push_back(std::move(b));
}

void BridgeManager::register_dds_to_ros_bridge(std::shared_ptr<dds_to_ros::DdsToRosBridge> b)
{
    dds_to_ros_bridges_.push_back(std::move(b));
}

bool BridgeManager::init_all()
{
    // create a single DomainParticipant and Publisher for all bridges
    // include robot id in participant name
    const char* env_id = std::getenv("ROBOT_ID");
    if (env_id == nullptr || env_id[0] == '\0')
    {
        std::cerr << "FATAL: environment variable ROBOT_ID is not set or empty. Please set ROBOT_ID and restart." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    std::string participant_name = std::string(env_id) + "_RosDdsBridge_Participant";

    DomainParticipantQos qos;
    qos.name(participant_name);
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, qos);
    if (!participant_)
    {
        std::cerr << "BridgeManager: Failed to create DomainParticipant" << std::endl;
        return false;
    }

    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (!publisher_)
    {
        std::cerr << "BridgeManager: Failed to create Publisher" << std::endl;
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
        participant_ = nullptr;
        return false;
    }

    // give ROS->DDS bridges the shared context and initialize them
    for (auto &b : ros_to_dds_bridges_)
    {
        b->set_dds_context(participant_, publisher_, std::string(env_id));
        if (!b->init())
        {
            std::cerr << "BridgeManager: ros->dds bridge init failed" << std::endl;
            return false;
        }
    }

    // For DDS->ROS bridges we might need a Subscriber instead of Publisher.
    // For now we leave Subscriber creation to individual bridges; in the future
    // we can create a shared Subscriber similar to Publisher.
    for (auto &b : dds_to_ros_bridges_)
    {
        b->set_dds_context(participant_, nullptr, std::string(env_id));
        if (!b->init())
        {
            std::cerr << "BridgeManager: dds->ros bridge init failed" << std::endl;
            return false;
        }
    }

    return true;
}
