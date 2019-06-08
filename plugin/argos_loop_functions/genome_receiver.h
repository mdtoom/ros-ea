//
// Created by matthijs on 8-6-19.
//

#ifndef MA_EVOLUTION_GENOME_RECEIVER_H
#define MA_EVOLUTION_GENOME_RECEIVER_H

#include <queue>
#include "ros/ros.h"
#include "message_decoder.h"
#include "../argos_ros_bot/robot_controller.h"

/** This class subscribes to /genome_topic and receives and stores the genomes in a queue. */
template <class T> class CGenomeReceiver
{
public:

    CGenomeReceiver(ros::NodeHandle* nodeHandle)
    {
        std::stringstream genome_topic_str;
        genome_topic_str << nodeHandle->getNamespace() << "/genome_topic";
        m_pcGenomeSub = nodeHandle->subscribe(genome_topic_str.str(), 1000, &CGenomeReceiver::receive_genome, this);
    }

    /**
     * This is a callback function for when a genome is received.
     * @param msg       - message containing the genome.
     */
    void receive_genome(const T& msg)
    {
        CRobotController *controller = decode_genome(msg);
        m_qControllerQueue.push(controller);

        LOG << "Received genome" << std::endl;
        LOG.Flush();
    }

    bool has_next()
    {
        return !m_qControllerQueue.empty();
    }

    /**
     * Returns the next controller from the list and removes it from its internal queue.
     * @return      - Next controller to be processed.
     */
    CRobotController *next()
    {
        CRobotController *next_controller = m_qControllerQueue.front();
        m_qControllerQueue.pop();
        return next_controller;
    }

private:

    /** This queue holds all controllers that need to be evaluated. */
    std::queue<CRobotController*> m_qControllerQueue;

    /** This subscribes ensures receiving the genomes. */
    ros::Subscriber m_pcGenomeSub;

};


#endif //MA_EVOLUTION_GENOME_RECEIVER_H
