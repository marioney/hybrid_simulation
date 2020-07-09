# hybrid_simulation package

======
 This code presents an open source tool for simulating autonomous vehicles in complex, high traffic, scenarios.

 The hybrid simulation fully integrates and synchronizes SUMO a microscopic, multi-modal traffic simulator and GAZEBO a complex 3D simulator.

## Video

Experiments for the hybrid simulation tool in a merge scenario with very high traffic.

[![hybrid simulation tool for very high traffic](http://img.youtube.com/vi/Xx5OmV86CsM/0.jpg)](https://youtu.be/Xx5OmV86CsM)

## Requisites

The hybrid simulation tested on the following software

* Ubuntu 18.04
* ROS Melodic
* Gazebo version 9.0.0
* SUMO Version 0.32.0

All of them can be installed using the standard installation process.

### Additional requirements

Make sure all ROS dependencies are installed.

   ```console
      cd [your-catkin-workspace]
      rosdep install -y -r -q --from-paths src --ignore-src --rosdistro melodic
   ```

#### The example provided uses as ego-vehicle the Demo of Prius in ROS/GAZEBO provided by the osrf.  It can be found in [https://github.com/osrf/car_demo](https://github.com/osrf/car_demo)

## How to compile

The code compiles only using *catkin_make*.

* cd to catkin workspace

   ```console
      cd [your-catkin-workspace
      catkin_make hybrid_simulation
   ```

## Demo the code

* Run `roslaunch hybrid_simulation hybrid_simulation.launch`
* Rviz is going to launch. You can enable the images (cameras)
* Wait till everything is launched and loaded in Gazebo and start the hybrid simulation by clicking the play button in SUMO.

## Executables and libraries

This package contains one executable for interfacing with SUMO and one Gazebo plugin.

### Gazebo plugin

The gazebo plugin SumoWorldPlugin is in charge of controlling the position of the vehicles in gazebo.

It needs to be added to the gazebo world. This is done by adding the following lines to the gazebo world file.

```xml
 <!-- World plugin for controlling sumo vehicles -->
 <plugin name="SumoUpdate" filename="libSumoWorldPlugin.so">
    <ego_vehicle_model_name>prius</ego_vehicle_model_name>
 </plugin>
```

This plugin controls the position of all additional vehicles. It has only one  parameter (ego_vehicle_model), it must be the same as the one set as the ROS parameter /ego_vehicle_name

### SUMO Control interface

The *control_other_vehicles.py* executable is a standalone executable that interfaces with SUMO and controls the execution of the traffic simulation, and it allows the message passing towards Gazebo.

## Launch files

It is suggested to run the launch files than to run the bin files

* `hybrid_simulation.launch`  - Main launcher with full simulation
* `spawn_prius.launch`        - Spawns the Prius model in Gazebo
* `sumo_interface.launch`     - Launches the SUMO simulator and the interface to communicate with gazebo.

## Brief explanation of the package contents

### src / sumo_world_plugin.cpp

Source code for the Gazebo plugin

### scripts / control_other_vehicles.py

Main source for the SUMO control interface

### src / hybrid_simulation / ego_vehicle.py

Source code for the EgoVehicle class: sets the status of ego-vehicle in SUMO with the values from Gazebo

### src / hybrid_simulation / route_file.py

Source code for the generation of the SUMO route file *network_traci.rou.xml*

### src / hybrid_simulation / traci_controls.py

Source code for auxiliary classes to control and interface with the simulation.

### sumo_files / network.sumocfg

Simulation and overall configuration file for the SUMO simulation.

### sumo_files / network.net.xmlt  

Description of the scenario, roads, intersections and other elements. It was created using the NETEDIT tool included in the SUMO installation.

### sumo_files / network.det.xml

Definition of a detector to control an intelligent traffic light.

### launchers

Different launchers detailed above.

### meshes / Car.dae

Mesh file for the car model used for all additional vehicles.

### sdf / models /car / car_model.sdf

SDF file for the car model used for all additional vehicles.

### worlds / roads.world

World file, with the scenario and Roads description for Gazebo. Includes the SumoWorldPlugin.