add_library(robot_controllers SHARED
        robot_controller.h
        perceptron_network.h perceptron_network.cpp
        state_machine_controller.h state_machine_controller.cpp
        neat_controller.h neat_controller.cpp
        transition_state_machine.h transition_state_machine.cpp
        selector_state_machine.h selector_state_machine.cpp
        fixed_two_state_controller.h fixed_two_state_controller.cpp
        right_hand_controller.h right_hand_controller.cpp
        )