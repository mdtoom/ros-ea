//
// Created by matthijs on 8-6-19.
//

#ifndef MA_EVOLUTION_MESSAGE_DECODER_H
#define MA_EVOLUTION_MESSAGE_DECODER_H

#include "../argos_ros_bot/robot_controller.h"
#include "ma_evolution/SMGenome.h"
#include "ma_evolution/SMSGenome.h"
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

/**
 * This function decodes a SMSGenome (state machine selected) message into a valid controller.
 * @param msg       - Message that contains the genome.
 * @return          - Controller that can be executed on the robot.
 */
CRobotController *decode_genome(const ma_evolution::SMSGenome& msg);

/**
 * This function decodes a fixed 2 state message, where the controller switches states when an object is to close.
 * @param msg       - Genome message
 * @return          - Controller build of genome message.
 */
CRobotController *decode_fixed_2_states(const ma_evolution::SMGenome& msg);


#endif //MA_EVOLUTION_MESSAGE_DECODER_H
