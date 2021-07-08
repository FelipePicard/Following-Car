# README

## Description
This is the last project for Nautilus' recruitment process. The goal is to create a program to locate, track and follow a robot that moves around with a red cylinder on top. Our robot has actuators, a camera, sonar (that gives the relative distance between the robots) and an odometer. Everything can be seen on a simulation program called Gazebo.

## How it works
We have written two main scripts: `cvBridge.py` and `controler.py`. They get the data from the sensors and talk with one another using ROS publishers and subscribers. Using our odometer we start out by moving around the perimeter of the main region of the simulation. Using the camera we selectively look for the red color of the cylinder we want to chase. Once it is spotted we stop searching and start following. The idea is that we want the robot with the cylinder to be always right in front of us, so we have an algorithm that repositions us.

## How to run it
In order to run this code, you should have ROS Melodic installed. Create a directory on your PC. Afterwards, once inside the created directory, create a new directory called `src` and clone this repository inside of it. Now go back to the directory you created initially and type the following command on your terminal `catkin_make`. After that you shall type `source devel/setup.bash`. 
With that done, you should be able to run the simulation with `roslaunch nemo_simulator gazebo.launch`. If something doesn't work, it is a good idea to go to the scrips and launch folders and type `chmod +x <filename.extention>`. After that, in another terminal, go to the firstly created folder and type `source devel/setup.bash`. Now the main code that controls the robot can be run using `rosrun nemo_simulator controler.py`. The same process can be done to run the cylinder robot, but the ROS command sould be `rosbag play nemo_vel.bag`. Our robot position can be reset by going to the folder where `resetpos.sh` is located and typing `./resetpos.sh`.
