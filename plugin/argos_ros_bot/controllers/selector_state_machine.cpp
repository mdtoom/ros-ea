//
// Created by matthijs on 18-6-19.
//

#include "selector_state_machine.h"


CSelectorState::CSelectorState(int state_id, const CPerceptronNetwork pn, const CPerceptronNetwork selector_pn)
        : CControllerState(state_id, pn), m_cSelectorNetwork(selector_pn)
{ }

int CSelectorState::next_state(const std::vector <Real> inputs) {

    std::vector<Real> state_scores = m_cSelectorNetwork.activate(inputs);

    // Find the maximal score and return its index.
    int max_index = 0;
    Real max_value = -1; // -1 is valid starting value because of tanh function

    for (int i = 0; i < state_scores.size(); i++)
    {
        if (state_scores[i] > max_value)
        {
            max_value = state_scores[i];
            max_index = i;
        }
    }

    return max_index;
}