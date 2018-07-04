hybrid_simulation package
====== 
 This code presents an open source tool for simulating autonomous vehicles in complex, high traffic, scenarios. 
 
 The hybrid simulation fully integrates and synchronizes SUMO a microscopic, multi-modal traffic simulator and GAZEBO a complex 3D simulator.

## Video

Experiments for the hybrid simulation tool in a merge scenario with very high traffic.

[![hybrid simulation tool for very high traffic](http://img.youtube.com/vi/Xx5OmV86CsM/0.jpg)](https://youtu.be/Xx5OmV86CsM)

## Requisites

The hybrid simulation tested on the following software

* Ubuntu 16.04
* ROS Kinetic
* Gazebo version 8.6.0
* SUMO Version 0.32.0

All of them can be installed usin the standard installation process.

#### ROS Dependencies are listed on the package.xml file

## How to compile

The code compiles only using *catkin_make*.

* cd to catkin workspace
* catkin_make hybrid_simulation

## Demo the code

 * Run ```roslaunch hybrid_simulation hybrid_simulation.launch```
 * Rviz is going to launch. You can enable the images (cameras)
 * Wait till everything is launched and loaded in Gazebo and start the SUMO simulation: Click the play button in SUMO.
 
## Executables and libraries

This package contains one executable for interfacing with SUMO and one Gazebo plugin.

### Gazebo plugin

The gazebo plugin SumoWorldPlugin is in charge of controlling the position of the vehicles in gazebo. 

It needs to be added to the gazebo world. This is done by adding the following lines to the gazebo world file. 


```
 <!-- World plugin for controlling sumo vehicles -->
 <plugin name="SumoUpdate" filename="libSumoWorldPlugin.so">
    <ego_vehicle_model_name>prius</ego_vehicle_model_name>
 </plugin>

```

This plugin controls the position of all additional vehicles. It has only one  parameter (ego_vehicle_model), it must be the same as the one set as the ROS parameter /ego_vehicle_name

### SUMO Control interface

The *control_other_vehicles.py* executable is a standalone executable that interfaces with SUMO and controls the execution of the traffic simulation, and it allows the message passing towards Gazebo.

## Launch files. 

It is suggested to run the launch files than to run the bin files

* hybrid_simulation.launch - Full simulation 


## Brief explanation of the package contents

#### scripts
#### src
#### sumo_files
#### launchers
#### meshes
#### sdf
#### worlds

## Acknowledgements





