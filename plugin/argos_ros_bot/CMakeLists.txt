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

add_library(differential_drive_robot SHARED
        differential_drive_robot.cpp differential_drive_robot.h
        )

target_link_libraries(differential_drive_robot
        robot_controllers
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

add_library(argos_ros_bot SHARED
        argos_ros_bot.h argos_ros_bot.cpp)

target_link_libraries(argos_ros_bot
        differential_drive_robot
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)

add_library(puck_seeing_ros_bot SHARED
        puck_seeing_ros_bot.cpp puck_seeing_ros_bot.h)

target_link_libraries(puck_seeing_ros_bot
        differential_drive_robot
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
