
# Add here the loop functions for specific examples.
add_library(mpga_phototaxis_loop_functions SHARED
  mpga_phototaxis_loop_functions.h mpga_phototaxis_loop_functions.cpp)

target_link_libraries(mpga_phototaxis_loop_functions
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)


add_library(mpga_phototaxis_obstacle_loop_functions SHARED
        mpga_phototaxis_obstacle_loop_functions.h mpga_phototaxis_obstacle_loop_functions.cpp)

target_link_libraries(mpga_phototaxis_obstacle_loop_functions
        mpga_phototaxis_loop_functions
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_media)
