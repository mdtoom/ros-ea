//
// Created by matthijs on 8-6-19.
//

#include "message_decoder.h"

#include "../argos_ros_bot/robot_controller.h"
#include "../argos_ros_bot/transition_state_machine.h"
#include "../argos_ros_bot/state_machine_controller.h"
#include "../argos_ros_bot/neat_controller.h"
#include "ma_evolution/WeightVector.h"
#include "ma_evolution/SMGenome.h"
#include "ma_evolution/SMState.h"
#include "ma_evolution/SMTransition.h"
#include "ma_evolution/SMCondition.h"
#include "ma_evolution/NEATGenome.h"
#include "ma_evolution/NNode.h"
#include "ma_evolution/NConnection.h"
#include <vector>


CRobotController *decode_genome(const ma_evolution::SMGenome& msg)
{
    std::vector<CControllerState*> states;

    // Generate all the states.
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

    CStateMachineController *new_controller = new CStateMachineController(msg.key, msg.gen_hash, states);

    // Add the transitions to the states.
    for (std::vector<ma_evolution::SMTransition>::const_iterator it = msg.transitions.begin();
            it != msg.transitions.end(); ++it)
    {
        CTransitionedState *state = (CTransitionedState*) new_controller->get_state(it->source);

        // Add all the conditions
        std::vector<CCondition> conditions;
        for (std::vector<ma_evolution::SMCondition>::const_iterator cit = it->conditions.begin();
             cit != it->conditions.end(); ++cit)
        {
            CCondition condition(cit->inputSensor, cit->comp_operator, cit->comparator_value);
            conditions.emplace_back(condition);
        }

        CStateTransition transition(it->source, it->dest, it->enabled, conditions);
        state->add_transition(transition);
    }

    return new_controller;
}

CRobotController *decode_genome(const ma_evolution::NEATGenome& msg)
{
    std::vector<CNeatNode> nodes;
    for (auto enc_node : msg.nodes)
    {
        nodes.emplace_back(CNeatNode(enc_node.key, enc_node.activation, enc_node.aggregation, enc_node.bias));
    }

    std::vector<CNeatConnection> connections;
    for (auto enc_conn : msg.connections)
    {
        connections.emplace_back(CNeatConnection(enc_conn.source, enc_conn.dest, enc_conn.enabled, enc_conn.weight));
    }

    return new CNeatNetwork(msg.key, msg.gen_hash, msg.num_outputs, connections, nodes);
}
