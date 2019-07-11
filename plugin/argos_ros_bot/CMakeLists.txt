add_subdirectory(controllers)

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
