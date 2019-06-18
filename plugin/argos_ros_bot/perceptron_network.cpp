//
// Created by matthijs on 7-6-19.
//

#include "perceptron_network.h"
#include <cmath>
#include <cassert>


Real sigmoid(Real input)
{
    return 1.0 / (1.0 + exp(-input));
}

Real linear(Real input)
{   // This mapping does not change the input value but just returns it.
    return input;
}

Real sum(std::vector<Real> input_vector)
{
    Real sum_of_elems = 0.0;
    for(std::vector<Real>::iterator it = input_vector.begin(); it != input_vector.end(); ++it)
        sum_of_elems += *it;

    return sum_of_elems;
}

activation_map create_activation_map()
{
    activation_map mapping;
    mapping["tanh"] = &tanh;
    mapping["sigmoid"] = &sigmoid;
    mapping["none"] = &linear;
    return mapping;
}

aggregation_map create_aggregation_map()
{
    aggregation_map mapping;
    mapping["sum"] = &sum;
    return mapping;
}

activation_map CPerceptronNetwork::s_mActivationMapping = create_activation_map();
aggregation_map CPerceptronNetwork::s_mAggregationMapping = create_aggregation_map();

CPerceptronNetwork::CPerceptronNetwork(std::vector<std::vector<Real>> weights, std::vector<Real> biases,
                                       const std::string activation_function, const std::string aggregation_function)
                                       : m_vWeights(weights), m_vBiases(biases)
{
    assert(m_vWeights.size() == m_vBiases.size());
    m_fActivationFunc = CPerceptronNetwork::s_mActivationMapping[activation_function];
    m_fAggregationFunc = CPerceptronNetwork::s_mAggregationMapping[aggregation_function];
}

std::vector<Real> CPerceptronNetwork::activate(const std::vector<Real> inputs)
{
   assert(m_vWeights[0].size() == inputs.size());
   std::vector<Real> outputs;
   for (int output_nr = 0; output_nr < m_vBiases.size(); output_nr++)
   {    // Go over all outputs (which is the number of biases.

        std::vector<Real> weightedValues;
        for (int input_nr = 0; input_nr < inputs.size(); input_nr++)
        {
            weightedValues.emplace_back(inputs[input_nr] * m_vWeights[output_nr][input_nr]);
        }

        Real aggregated_value = m_fAggregationFunc(weightedValues) + m_vBiases[output_nr];
        Real activated_value = m_fActivationFunc(aggregated_value);
        outputs.emplace_back(activated_value);
   }

   return outputs;
}