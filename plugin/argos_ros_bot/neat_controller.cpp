//
// Created by matthijs on 11-6-19.
//

#include "neat_controller.h"
#include "perceptron_network.h"
#include <cassert>

CNeatConnection::CNeatConnection(int source, int destination, bool enabled, Real weight)
    : m_iSource(source), m_iDestination(destination), m_bEnabled(enabled), m_fWeight(weight)
{ }

CNeatNode::CNeatNode(int key, std::string activation_function, std::string aggregation_function, Real bias)
    : m_iKey(key), m_fBias(bias)
{
    m_fActivationFunc = CPerceptronNetwork::s_mActivationMapping[activation_function];
    m_fAggregationFunc = CPerceptronNetwork::s_mAggregationMapping[aggregation_function];
}

void CNeatNode::add_incoming_connection(CNeatConnection connection) {
    m_vDependencies.emplace_back(connection);
}

bool CNeatNode::evaluate(std::map<int, Real> &evaluated_nodes) {

    // First check that all dependencies are met.
    for (auto& dependency : m_vDependencies)
    {
        if (evaluated_nodes.find(dependency.m_iSource) == evaluated_nodes.end())
        {
            return false;
        }
    }

    // If all dependencies are met evaluate the node and add the result to the map.
    std::vector<Real> weightedValues;
    for (auto& dependency : m_vDependencies)
    {
        weightedValues.emplace_back(dependency.m_fWeight * evaluated_nodes[dependency.m_iSource]);
    }

    Real aggregated_value = m_fAggregationFunc(weightedValues) + m_fBias;
    Real activated_value = m_fActivationFunc(aggregated_value);
    evaluated_nodes[m_iKey] = activated_value;

    return true;
}

CNeatNetwork::CNeatNetwork(int id, int gen_id, int num_outputs, std::vector<CNeatConnection> connections,
        std::vector<CNeatNode> nodes)
        : CRobotController(id, gen_id), m_iNumOutputs(num_outputs)
{
    // Set all the nodes.
    for (std::vector<CNeatNode>::iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        m_vNodes.insert(std::pair<int, CNeatNode>(it->m_iKey, *it));
    }

    // Set all the connections as dependencies of for nodes.
    for (std::vector<CNeatConnection>::iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->m_bEnabled)
        {   // If the node is enabled, add it as a dependency to the correct node.
            assert(m_vNodes.find(it->m_iDestination) != m_vNodes.end());
            m_vNodes.find(it->m_iDestination)->second.add_incoming_connection(*it);
        }
    }

    // Check that for every output there is a node available.
    for (int i = 0; i < num_outputs; i++)
    {
        assert(m_vNodes.find(i) != m_vNodes.end());
    }
}

std::vector<Real> CNeatNetwork::activate(std::vector<Real> inputs)
{
    std::map<int, Real> evaluated_nodes;

    // Set the values of the initial nodes.
    for (int sensor_nr = 0; sensor_nr < inputs.size(); sensor_nr++)
    {
        evaluated_nodes[-sensor_nr - 1] = inputs[sensor_nr];
    }

    bool not_found_all = true;
    bool evaluated_one;
    do {
        evaluated_one = false; // Ensure that every iteration, at least one node is evaluated, if not endless loop.

        // Evaluate all the nodes that currently can be evaluated.
        for (auto& node : m_vNodes)
        {
            evaluated_one |= node.second.evaluate(evaluated_nodes);
        }

        not_found_all = false;      // This variable becomes one if any of the outputs is not found.

        for (int i = 0; i < m_iNumOutputs; i++)
        {
            not_found_all |= evaluated_nodes.find(i) == evaluated_nodes.end();
        }

#ifdef DEBUG
        std::cout << "[";
        for (auto& node : evaluated_nodes)
        {
            std::cout << node.first << ": " << node.second << ", ";
        }
        std::cout << "]" << std::endl;
#endif

        std::cout << "l";

    } while(not_found_all && evaluated_one);

    std::cout << std::endl;

    // Store all the outputs in the return vector.
    std::vector<Real> outputs;
    for (int i = 0; i < m_iNumOutputs; i++)
    {
        outputs.emplace_back(evaluated_nodes[i]);
    }

    return outputs;
}
