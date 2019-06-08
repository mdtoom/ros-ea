//
// Created by matthijs on 8-6-19.
//

#ifndef MA_EVOLUTION_MESSAGE_DECODER_H
#define MA_EVOLUTION_MESSAGE_DECODER_H

#include "../argos_ros_bot/robot_controller.h"
#include "ma_evolution/SMGenome.h"
#include "ma_evolution/NEATGenome.h"

/**
 * This function decodes a SMGenome message into a valid controller.
 * @param msg       - Message that contains the genome.
 * @return          - Controller that can be executed.
 */
CRobotController *decode_genome(const ma_evolution::SMGenome& msg);

/**
 * This function decodes a NEATGenome message into a valid controller.
 * @param msg       - Message that contains the genome.
 * @return          - Controller that can be executed.
 */
CRobotController *decode_genome(const ma_evolution::NEATGenome& msg);

#endif //MA_EVOLUTION_MESSAGE_DECODER_H
