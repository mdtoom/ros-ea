#include "perceptron_network_test.h"
#include "state_machine_test.h"
#include "neat_test.h"

int main()
{
    perceptron_network_test_1();
    perceptron_network_test_2();
    state_machine_test_1_state();
    condition_test();
    transition_test();
    transitioned_state_test();
    neat_state_test();
    neat_algorithm_test();
    return 0;
}

