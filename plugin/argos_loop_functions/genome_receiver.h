//
// Created by matthijs on 8-6-19.
//

#ifndef MA_EVOLUTION_GENOME_RECEIVER_H
#define MA_EVOLUTION_GENOME_RECEIVER_H

#include <queue>
#include "ros/ros.h"
#include "message_decoder.h"
#include "../argos_ros_bot/controllers/robot_controller.h"

/** This class stores genomes that are ready to be evaluated. */
class CGenomeBuffer
{
public:
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

protected:

    /** This queue holds all controllers that need to be evaluated. */
    std::queue<CRobotController*> m_qControllerQueue;
};

/** This class subscribes to /genome_topic and receives and stores the genomes in a queue. */
template <class T> class CGenomeReceiver : public CGenomeBuffer
{
public:

    CGenomeReceiver(ros::NodeHandle* nodeHandle) : m_iCurrentGenHash(-1), m_fDecodeFunction(&decode_genome)
    {
        std::stringstream genome_topic_str;
        genome_topic_str << nodeHandle->getNamespace() << "/genome_topic";
        m_pcGenomeSub = nodeHandle->subscribe(genome_topic_str.str(), 1000, &CGenomeReceiver::receive_genome, this);
    }


    CGenomeReceiver(ros::NodeHandle* nodeHandle, CRobotController *(*decode_function)(const T&))
        : CGenomeReceiver(nodeHandle)
    {
        m_fDecodeFunction = decode_function;
    }



protected:

    /**
     * This is a callback function for when a genome is received.
     * @param msg       - message containing the genome.
     */
    void receive_genome(const T& msg)
    {
        if (msg.gen_hash != m_iCurrentGenHash)
        {
            // Clearing the queue
            while (!m_qControllerQueue.empty())
            {
                m_qControllerQueue.pop();
            }
            m_iCurrentGenHash = msg.gen_hash;
            LOG << "Different gen hash received, clearing queue" << std::endl;
        }

        CRobotController *controller = m_fDecodeFunction(msg);
        m_qControllerQueue.push(controller);
    }

private:

    /** This subscribes ensures receiving the genomes. */
    ros::Subscriber m_pcGenomeSub;

    int m_iCurrentGenHash;

    CRobotController *(*m_fDecodeFunction)(const T&);

};


#endif //MA_EVOLUTION_GENOME_RECEIVER_H
