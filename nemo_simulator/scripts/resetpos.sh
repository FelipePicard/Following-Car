#!/bin/bash
# this script will jump the model named "marlin" to the desired position
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'marlin'
  pose:
    position:
      x: 5
      y: 7
      z: 0.23
    orientation:
      x: 0.0
      y: 0.0
      z: 0
      w: 1"