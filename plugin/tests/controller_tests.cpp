#include <cassert>
#include <vector>
#include "../argos_ros_bot/perceptron_network.h"

void perceptron_network_test_1()
{

    std::vector<Real> weight_1{1.0};
    std::vector<Real> weight_2{-0.5};
    std::vector<std::vector<Real>> weights{weight_1, weight_2};
    std::vector<Real> biases{0.0, 1.0};
    std::string actfunc = "tanh";
    std::string aggfunc = "sum";

    CPerceptronNetwork pn(weights, biases, actfunc, aggfunc);

    // Test 1: put 0 in.
    std::vector<Real> inputs{0.0};
    std::vector<Real> results = pn.activate(inputs);

    assert(results.size() == 2);
    assert(results[0] == 0.0);
    assert(results[1] > 0.76 && results[1] < 0.77);

    // Test 2: put 1 in.
    inputs.clear();
    inputs.emplace_back(1.0);
    results = pn.activate(inputs);
    assert(results[0] > 0.76 && results[0] < 0.77);
    assert(results[1] > 0.46 && results[1] < 0.47);

    // Test 2: put -1.5 in.
    inputs.clear();
    inputs.emplace_back(-1.5);
    results = pn.activate(inputs);
    assert(results[0] > -0.91 && results[0] < -0.9);
    assert(results[1] > 0.94 && results[1] < 0.95);
}

void perceptron_network_test_2()
{

    std::vector<Real> weight_1{1.0, -0.5};
    std::vector<std::vector<Real>> weights{weight_1};
    std::vector<Real> biases{0.5};
    std::string actfunc = "tanh";
    std::string aggfunc = "sum";

    CPerceptronNetwork pn(weights, biases, actfunc, aggfunc);

    // Test 1: put 0 in.
    std::vector<Real> inputs{0.0, 0.0};
    std::vector<Real> results = pn.activate(inputs);
    assert(results.size() == 1);
    assert(results[0] > 0.46 && results[0] < 0.47);

    // Test 2: put 1 in.
    inputs.clear();
    inputs.emplace_back(1.0);
    inputs.emplace_back(1.0);
    results = pn.activate(inputs);
    assert(results[0] > 0.76 && results[0] < 0.77);

    // Test 2: put -1.5 in.
    inputs.clear();
    inputs.emplace_back(0.0);
    inputs.emplace_back(1.0);
    results = pn.activate(inputs);
    assert(results[0] == 0);
}

int main()
{
    perceptron_network_test_1();
    perceptron_network_test_2();
    return 0;
}

