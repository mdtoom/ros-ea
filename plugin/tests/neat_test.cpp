//
// Created by matthijs on 12-6-19.
//

#include "neat_test.h"
#include <cassert>
#include <iostream>
#include <map>
#include "../argos_ros_bot/controllers/neat_controller.h"

const ControllerHeader header{0, 0, 0};

void neat_state_test()
{
    std::string actfunc = "tanh";
    std::string aggfunc = "sum";
    std::map<int, Real> evaluated_nodes;

    CNeatNode node(1, actfunc, aggfunc, 1.0);
    node.evaluate(evaluated_nodes);
    assert(evaluated_nodes[1] > 0.76 && evaluated_nodes[1] < 0.77);

    // Check that a not evaluated node does cause this node not to be evaluated.
    evaluated_nodes.clear();
    node.add_incoming_connection(CNeatConnection(0, 1, false, 0.5));
    node.evaluate(evaluated_nodes);
    assert(evaluated_nodes.size() == 0);

    // Add a different node
    evaluated_nodes[2] = 1.5;
    node.evaluate(evaluated_nodes);
    assert(evaluated_nodes.size() == 1);
    assert(evaluated_nodes.find(1) == evaluated_nodes.end());

    // Add the correct node and check that it is evaluated.
    evaluated_nodes[0] = 2.0;
    node.evaluate(evaluated_nodes);
    assert(evaluated_nodes.size() == 3);
    assert(evaluated_nodes[1] > 0.96 && evaluated_nodes[1] < 0.97);

    // Add another connection and check that does not evaluate;
    evaluated_nodes.clear();
    evaluated_nodes[0] = 2.0;
    node.add_incoming_connection(CNeatConnection(3, 1, false, -0.5));
    node.evaluate(evaluated_nodes);
    assert(evaluated_nodes.find(1) == evaluated_nodes.end());

    // Add the correct values and ensure that it does evaluate.
    evaluated_nodes[3] = 4.0;
    node.evaluate(evaluated_nodes);
    assert(evaluated_nodes[1] == 0.0);

    std::cout << "NEAT state test passed." << std::endl;
}

void neat_node_sequence_test()
{
    // This test tests a sequence of nodes.

    std::string actfunc = "tanh";
    std::string aggfunc = "sum";

    // Test 2 states in sequence
    std::vector<CNeatNode> nodes{CNeatNode(0, actfunc, aggfunc, 1.0),
                                 CNeatNode(10, actfunc, aggfunc, 0.0)};
    std::vector<CNeatConnection> connections{
        CNeatConnection(-1, 10, true, 1.0),
        CNeatConnection(10, 0, true, -0.5)
    };
    CNeatNetwork nn(header, 1, connections, nodes);
    std::vector<Real> outputs = nn.activate({1.0});
    assert(outputs.size() == 1);
    assert(outputs[0] > 0.55 && outputs[0] < 0.56);
}

void multiple_output_nodes_test()
{
    std::string actfunc = "tanh";
    std::string aggfunc = "sum";

    // Test 2 states in sequence
    std::vector<CNeatNode> nodes{CNeatNode(0, actfunc, aggfunc, 1.0),
                                 CNeatNode(1, actfunc, aggfunc, 0.0)};
    std::vector<CNeatConnection> connections{
            CNeatConnection(-1, 0, true, 1.0),
            CNeatConnection(-1, 1, true, -0.5),
            CNeatConnection(-2, 0, true, -1.0),
            CNeatConnection(-2, 1, true, 0.5)
    };
    CNeatNetwork nn(header, 2, connections, nodes);
    std::vector<Real> outputs = nn.activate({1.0, 0.5});
    assert(outputs.size() == 2);
    assert(outputs[0] > 0.9 && outputs[0] < 0.91);
    assert(outputs[1] > -0.25 && outputs[1] < -0.24);
}

void filter_disabled_connection_test()
{
    std::string actfunc = "tanh";
    std::string aggfunc = "sum";

    // Test 2 states in sequence
    std::vector<CNeatNode> nodes{CNeatNode(0, actfunc, aggfunc, 1.0),
                                 CNeatNode(1, actfunc, aggfunc, 0.0)};
    std::vector<CNeatConnection> connections{
            CNeatConnection(-1, 0, false, 1.0),
            CNeatConnection(-1, 1, false, -0.5),
            CNeatConnection(-2, 0, true, -1.0),
            CNeatConnection(-2, 1, true, 0.5)
    };
    CNeatNetwork nn(header, 2, connections, nodes);
    std::vector<Real> outputs = nn.activate({1.0, 0.5});

    assert(outputs[0] > 0.46 && outputs[0] < 0.47);
    assert(outputs[1] > 0.24 && outputs[1] < 0.25);
}

void additional_connection_test()
{
    std::string actfunc = "none";
    std::string aggfunc = "sum";

    // Test 2 states in sequence
    std::vector<CNeatNode> nodes{CNeatNode(0, actfunc, aggfunc, 1.0),
                                 CNeatNode(1, actfunc, aggfunc, 0.0),
                                 CNeatNode(2, actfunc, aggfunc, 0.0),
                                 CNeatNode(3, actfunc, aggfunc, 0.0)};
    std::vector<CNeatConnection> connections{
            CNeatConnection(-1, 2, true, 1.0),
            CNeatConnection(-1, 3, true, 1.5),
            CNeatConnection(-2, 3, true, -0.5),
            CNeatConnection(3, 0, true, -1.0),
            CNeatConnection(3, 1, true, 0.5),
            CNeatConnection(2, 0, true, 0.5)
    };
    CNeatNetwork nn(header, 2, connections, nodes);
    std::vector<Real> outputs = nn.activate({1.0, 0.5});

    assert(outputs[0] > 0.24 && outputs[0] < 0.26);
    assert(outputs[1] > 0.6 && outputs[1] < 0.65);

}

void neat_algorithm_test()
{
    neat_node_sequence_test();
    multiple_output_nodes_test();
    filter_disabled_connection_test();
    additional_connection_test();

    std::cout << "NEAT algorithm test passed" << std::endl;
}