add_library(robot_controllers SHARED
        robot_controller.h
        perceptron_network.h perceptron_network.cpp
        state_machine_controller.h state_machine_controller.cpp
        neat_controller.h neat_controller.cpp
        transition_state_machine.h transition_state_machine.cpp
        selector_state_machine.h selector_state_machine.cpp
        )

add_library(argos_ros_bot SHARED argos_ros_bot.h argos_ros_bot.cpp)

target_link_libraries(argos_ros_bot
  robot_controllers
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

