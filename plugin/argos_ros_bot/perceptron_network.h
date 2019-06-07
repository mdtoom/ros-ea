//
// Created by matthijs on 7-6-19.
//

#ifndef MA_EVOLUTION_PERCEPTRON_NETWORK_H
#define MA_EVOLUTION_PERCEPTRON_NETWORK_H

#include <vector>
#include <string>
#include <map>
#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

// These typedefs simplify usage of activation map.
typedef std::map<std::string, Real (*)(Real)> activation_map;
typedef std::map<std::string, Real (*)(std::vector<Real>)> aggregation_map;

class CPerceptronNetwork {

public:

    CPerceptronNetwork(std::vector<std::vector<Real>> weights, std::vector<Real> biases,
            const std::string activation_function, const std::string aggregation_function);

    /**
     * This function activates the perceptron network with the given inputs.
     * @return      - vector containing the outputs of the calculation.
     */
    std::vector<Real> activate(const std::vector<Real> inputs);

    /** Map containing all activation functions available. (initialized in perceptron_network.cpp) */
    static activation_map s_mActivationMapping;

    /** Map containing all aggregation functions available. (initialized in perceptron_network.cpp) */
    static aggregation_map s_mAggregationMapping;

private:

    std::vector<std::vector<Real>> m_vWeights;
    std::vector<Real> m_vBiases;

    /** This function aggregates the values that end up at a single node. */
    Real (*m_fAggregationFunc)(std::vector<Real>);

    /** This function applies the activation function to the result of the aggregation. */
    Real (*m_fActivationFunc)(Real);
};


#endif //MA_EVOLUTION_PERCEPTRON_NETWORK_H
