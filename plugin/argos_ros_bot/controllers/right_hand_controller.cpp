//
// Created by matthijs on 25-6-19.
//

#include "right_hand_controller.h"
#include "perceptron_network.h"

CRightHandBot::CRightHandBot()
    : CRobotController(0, 0), m_iState(0)
{}

std::vector<Real> CRightHandBot::activate(std::vector<Real> inputs)
{

    std::vector<Real> light_vector;
//    std::cout << "light: [";
    for (int i = 0; i < 12; i++)
    {
        light_vector.emplace_back(inputs[i + 12]);
//        std::cout << inputs[i + 12] << ", ";
    }
//    std::cout << "] -> [";

    std::vector<Real> obst_vector;
    std::cout << "obst: [";
    for (int i = 0; i < 12; i++)
    {
        obst_vector.emplace_back(inputs[i]);
        std::cout << inputs[i] << ", ";
    }
    std::cout << "] -> ";

    if (m_iState == 0 & (obst_vector[0] > 0.25 || obst_vector[1] > 0.25 || obst_vector[11] > 0.25))
    {
        std::cout << "State change to avoidance" << std::endl;
        m_iState = 1;
    }

    if (m_iState == 1 && (obst_vector[0] < 0.25 && obst_vector[1] < 0.25 && obst_vector[2] < 0.25 && obst_vector[3] < 0.25 && obst_vector[4] < 0.25))
    {
        std::cout << "State change to go to light" << std::endl;
        m_iState = 0;
    }

    if (m_iState == 0)
    {
        // Calculate score for light attraction.
        std::vector<Real> left_wheels_light{0.0, 0.0, 0.0, -0.5, -0.5, -0.5, 0.0, 0.5, 1.0, 1.5, 1.0, 0.5};
        std::vector<Real> right_wheels_light{0.5, 1.0, 1.5, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        CPerceptronNetwork light_attraction_nw({left_wheels_light, right_wheels_light}, {0.0, 0.0}, "tanh", "sum");
        std::vector<Real> light_outputs = light_attraction_nw.activate(light_vector);

        std::cout << "light:[";
        for (auto output : light_outputs)
        {
            std::cout << output << ", ";
        }
        std::cout << "]" << std::endl;

        return light_outputs;
    } else {
        // Calculate score for obstacle avoidance.
        std::vector<Real> left_wheels_obst{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<Real> right_wheels_obst{-1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5};
        CPerceptronNetwork obst_avoidance_attraction_pn({left_wheels_obst, right_wheels_obst}, {1.0, 1.0}, "tanh", "sum");
        std::vector<Real> outputs = obst_avoidance_attraction_pn.activate(obst_vector);

        std::cout << "avoid:[";
        for (auto output : outputs)
        {
            std::cout << output << ", ";
        }
        std::cout << "]" << std::endl;

        return outputs;
    }








}
