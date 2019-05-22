#!/bin/bash

rosrun ma_evolution sm_ga_runner_selector.py

rosparam set /sim1/controller feed-forward
rosparam set /sim2/controller feed-forward
rosparam set /sim3/controller feed-forward
rosparam set /sim4/controller feed-forward

rosservice call /sim1/update_params
rosservice call /sim2/update_params
rosservice call /sim3/update_params
rosservice call /sim4/update_params

rosrun ma_evolution one-layer-nn-runner.py
rosrun ma_evolution neat_ga_runner.py