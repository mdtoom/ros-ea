add_executable(controller_tests
        controller_tests.cpp
        perceptron_network_test.h perceptron_network_test.cpp
        state_machine_test.h state_machine_test.cpp)
target_link_libraries(controller_tests robot_controllers)