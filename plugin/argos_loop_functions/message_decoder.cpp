//
// Created by matthijs on 8-6-19.
//

#include "message_decoder.h"

#include "../argos_ros_bot/robot_controller.h"
#include "../argos_ros_bot/transition_state_machine.h"
#include "../argos_ros_bot/selector_state_machine.h"
#include "../argos_ros_bot/state_machine_controller.h"
#include "../argos_ros_bot/fixed_two_state_controller.h"
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
#include <cassert>

std::vector<std::vector<Real>> decode_weight_vector(const std::vector<ma_evolution::WeightVector>& weight_vector)
{
    std::vector<std::vector<Real>> weights;
    for (std::vector<ma_evolution::WeightVector>::const_iterator wit = weight_vector.begin();
         wit != weight_vector.end(); ++wit)
    {
        weights.emplace_back(std::vector<Real>(wit->weights));
    }

    return weights;
}


CRobotController *decode_genome(const ma_evolution::SMGenome& msg)
{
    std::vector<CControllerState*> states;

    // Generate all the states.
    for (std::vector<ma_evolution::SMState>::const_iterator it = msg.states.begin(); it != msg.states.end(); ++it)
    {
        // Copy the biases.
        std::vector<Real> biases(it->biases);

        // Copy the weights
        std::vector<std::vector<Real>> weights = decode_weight_vector(it->weight_vec);

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

        CStateTransition transition(it->source, it->dest, it->enabled, it->or_comparison, conditions);
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

CRobotController *decode_genome(const ma_evolution::SMSGenome& msg)
{
    std::vector<CControllerState*> states;

    // Generate all the states.
    for (auto &enc_state : msg.states)
    {
        // Copy the biases.
        std::vector<Real> biases(enc_state.biases);

        // Copy the weights
        std::vector<std::vector<Real>> weights = decode_weight_vector(enc_state.weight_vec);

        CPerceptronNetwork pn(weights, biases, enc_state.activation, enc_state.aggregation);

        // Now find the corresponding selector state and create a new selector state
        for (auto enc_selector_state : msg.selectors)
        {
            if (enc_selector_state.key == enc_state.key)
            {   // If the corresponding selector state is found create a new state.

                std::vector<std::vector<Real>> sel_weights = decode_weight_vector(enc_selector_state.weight_vec);
                CPerceptronNetwork selector_pn(sel_weights, enc_selector_state.biases,
                        enc_selector_state.activation, enc_selector_state.aggregation);

                CControllerState *new_state = new CSelectorState(enc_state.key, pn, selector_pn);
                states.emplace_back(new_state);
            }
        }
    }

    CStateMachineController *new_controller = new CStateMachineController(msg.key, msg.gen_hash, states);
    return new_controller;
}

CRobotController *decode_fixed_2_states(const ma_evolution::SMGenome& msg)
{
    assert(msg.states.size() == 2);

    // Decode the first state
    std::vector<std::vector<Real>> weights0 = decode_weight_vector(msg.states[0].weight_vec);
    CPerceptronNetwork pn0(weights0, msg.states[0].biases, msg.states[0].activation, msg.states[0].aggregation);
    CTransitionedState state1(msg.states[0].key, pn0);

    // Decode the second state
    std::vector<std::vector<Real>> weights1 = decode_weight_vector(msg.states[1].weight_vec);
    CPerceptronNetwork pn1(weights1, msg.states[1].biases, msg.states[1].activation, msg.states[1].aggregation);
    CTransitionedState state2(msg.states[1].key, pn1);

    // Create the new controller
    return new CFixedTwoStateController(msg.key, msg.gen_hash, state1, state2);
}
