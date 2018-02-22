# Path Planning

## 1. Introduction
This model implements a path planner in C++ using the splines library and waypoints provided by a simulator

## 2. Project Environment
The project was built using the Ubuntu 16-04 bash shell in Windows 10. Instructions to set this up can be found [here](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/). The following dependencies need to be in place to build and execute the project.

* [Udacity SDC Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)
* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4
* uWebSocketIO (installed via the [install-ubuntu.sh](https://github.com/shazraz/Path-Planning/blob/master/install-ubuntu.sh) script)

The project consists primarily of the following files located in the [src](https://github.com/shazraz/Path-Planning/tree/master/src) folder:

* [main.cpp](https://github.com/shazraz/Path-Planning/blob/master/src/main.cpp): Interfaces with the simulator using uWebSocketIO to recieve vehicle localization & sensor fusion data to generate the coordinates of the path to follow.
* [transforms.cpp](https://github.com/shazraz/Path-Planning/blob/master/src/transforms.cpp): Includes helper functions to transform from Cartesian to Frenet coordinates.

Once the environment is ready, the code can be tested as follows:

1. Execute ```./path_planning``` from the build directory
2. Launch the simulator and click Start

## 3. Project Data
### 3.1 Map data
The map of the highway is in data/highway_map.txt. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### 3.2 Simulator Data

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.]

## 4. Path Planning
