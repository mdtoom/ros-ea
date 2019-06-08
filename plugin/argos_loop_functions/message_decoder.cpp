//
// Created by matthijs on 8-6-19.
//

#include "message_decoder.h"

#include "../argos_ros_bot/robot_controller.h"
#include "../argos_ros_bot/transition_state_machine.h"
#include "../argos_ros_bot/state_machine_controller.h"
#include "ma_evolution/WeightVector.h"
#include <vector>


CRobotController *decode_genome(const ma_evolution::SMGenome& msg)
{
    std::vector<CControllerState*> states;

    std::cout << typeid(msg.states).name() << std::endl;

    for (std::vector<ma_evolution::SMState>::const_iterator it = msg.states.begin(); it != msg.states.end(); ++it)
    {

        // Copy the biases.
        std::vector<Real> biases(it->biases);

        // Copy the weights
        std::vector<std::vector<Real>> weights;
        for (std::vector<ma_evolution::WeightVector>::const_iterator wit = it->weight_vec.begin();
                wit != it->weight_vec.end(); ++wit)
        {
            weights.emplace_back(std::vector<Real>(wit->weights));
        }

        CPerceptronNetwork pn(weights, biases, it->activation, it->aggregation);
        CControllerState *new_state = new CTransitionedState(it->key, pn);
        states.emplace_back(new_state);
    }

    CRobotController *new_controller = new CStateMachineController(msg.key, msg.gen_hash, states);

    return new_controller;
    //TODO: add transition decoding.
}
