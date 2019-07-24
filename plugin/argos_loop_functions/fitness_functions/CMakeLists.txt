add_library(fitness_functions SHARED
        fitness_function.h fitness_function.cpp
        absolute_distance_fitness_function.h absolute_distance_fitness_function.cpp
        space_dependent_fitness_function.h space_dependent_fitness_function.cpp
        summed_in_light_range_fitness_function.h summed_in_light_range_fitness_function.cpp
        come_and_go_fitness_function.h come_and_go_fitness_function.cpp
        )

target_link_libraries(fitness_functions
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_media
        argos_ros_bot)