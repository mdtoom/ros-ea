//
// Created by matthijs on 11-6-19.
//

#ifndef MA_EVOLUTION_NEAT_NETWORK_H
#define MA_EVOLUTION_NEAT_NETWORK_H

#include "robot_controller.h"
#include <vector>
#include <map>

class CNeatConnection {

public:

    CNeatConnection(int source, int destination, bool enabled, Real weight);

    bool m_bEnabled;
    Real m_fWeight;
    int m_iSource;
    int m_iDestination;
};

class CNeatNode {

public:

    CNeatNode(int key, std::string activation_function, std::string aggregation_function, Real bias);

    int m_iKey;

    /** This function adds an incoming connection (a dependency) to the node. */
    void add_incoming_connection(CNeatConnection connection);

    /**
     * This function evaluates whether all dependencies are met, and if so adds the value of this node to the list
     * of evaluated nodes.
     * @param evaluated_nodes   - Map containing all nodes evaluated so far. if dependencies met, the value of this
     *                            will be added.
     */
    bool evaluate(std::map<int, Real> &evaluated_nodes);

private:

    /** This function aggregates the values that end up at a single node. */
    Real (*m_fAggregationFunc)(std::vector<Real>);

    /** This function applies the activation function to the result of the aggregation. */
    Real (*m_fActivationFunc)(Real);

    Real m_fBias;

    std::vector<CNeatConnection> m_vDependencies;  // This vector contains all de dependencies on the current node.
};

/** This class is able to execute a NEAT network, which is send to the simulator, as a robot controller. */
class CNeatNetwork : public CRobotController {

public:

    CNeatNetwork(int id, int gen_id, int num_outputs,
            std::vector<CNeatConnection> connections, std::vector<CNeatNode> nodes);

    ~CNeatNetwork();

    /** This function should execute the controller. */
    virtual std::vector<Real> activate(std::vector<Real> inputs);

private:

    std::map<int, CNeatNode> m_vNodes;

    int m_iNumOutputs;  // Returns the number of output nodes, this is important to return a value for all actuators.
};


#endif //MA_EVOLUTION_NEAT_NETWORK_H
