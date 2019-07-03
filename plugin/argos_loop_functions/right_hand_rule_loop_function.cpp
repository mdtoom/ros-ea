//
// Created by matthijs on 26-6-19.
//

#include "right_hand_rule_loop_function.h"
#include "../argos_ros_bot/argos_ros_bot.h"
#include "../argos_ros_bot/controllers/right_hand_controller.h"

void CRightHandRuleLoopFunction::Init(TConfigurationNode &t_node)
{
    CFitnessEvaluatingLoopFunction::Init(t_node);

    CArgosRosBot &controller = (CArgosRosBot&) m_pcFootBot->GetControllableEntity().GetController();
    controller.set_controller(new CRightHandBot());

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CRightHandRuleLoopFunction, "right_hand_rule_loop_functions")