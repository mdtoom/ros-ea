add_subdirectory(fitness_functions)

add_library(fitness_evaluating_loop_function SHARED
        robot_launching_loop_function.h robot_launching_loop_function.cpp
        fitness_evaluating_loop_function.h fitness_evaluating_loop_function.cpp)

target_link_libraries(fitness_evaluating_loop_function
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_media
        /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so
        argos_ros_bot
        fitness_functions
        )

add_library(genome_runner_loop_functions SHARED
        genome_runner_loop_function.h genome_runner_loop_function.cpp
        light_random_init_runner.cpp light_random_init_runner.h
        random_obstacles_loop_function.h random_obstacles_loop_function.cpp
        genome_receiver.h
        message_decoder.h message_decoder.cpp
        )

target_link_libraries(genome_runner_loop_functions
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_media
        /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so
        argos_ros_bot
        fitness_evaluating_loop_function)

add_library(right_hand_rule_loop_functions SHARED
        right_hand_rule_loop_function.h right_hand_rule_loop_function.cpp
        )



