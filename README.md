# Path Planning

## 1. Introduction
This model implements a path planner in C++ using vehicle localization and sensor fusion data provided by a simulator. The path is used to navigate the vehicle on a highway while avoiding obstacles on the road.

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

The data provided from the simulator is as follows:

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

["sensor_fusion"] A 2d vector of cars and then that car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## 4. Path Planning
The path planner generates a trajectory for the vehicle to follow using waypoints in a map. When the vehicle is initialized, a number of anchor points are created in global coordinates, defined by ```N_ANCHORS```. Two of these anchor points consist of the vehicles current state and a state shifted back in time corresponding to the vehicle headings. Once a previous trajectory is available, the last two points of the previous trajectory are used as the first two anchor points. The other three anchor points are projected into the path ahead using ```ANCHOR_SPACING```. 

The splines library is then used to fit a spline to these anchor points.  ```N_SPLINE_POINTS``` are then extracted from this spline upto a distance of ```path_horizon``` from the vehicle. These points form the vehicle trajectory after some coordinate transformations from vehicle to map coordinates. The use of trajectory points extracted from a spline allow for smooth navigation within a lane and when changing lanes.

The vehicle analyzes the sensor fusion data returned from the simulator to determine whether it is safe to perform a lane change. The sensor fusion data is parsed to identify vehicles detected around the ego vehicle. The positon of the detected vehicles, referred to as obstacles, is projected forward in time to the end of the ego vehicle's planned path. a lane change maneuver is considered dangerous if an obstacle is in an adjacent lane and less than ```MIN_SAFE_GAP``` ahead or ```MIN_SAFE_GAP/2``` behind the ego vehicle. In this case, the ```RIGHT_LANE_CLEAR``` or ```LEFT_LANE_CLEAR``` flags are set to False as applicable. The safe gap for vehicles behind the ego vehicle is reduced by a factor of 2 to allow quicker lane changes once an obstacle is crossed. A larger distance is maintained for obstacles in front of the ego vehicle to allow for headroom in case of emergency braking by vehicles ahead. 

When an obstacle is detected in the path of the vehicle and is at least ```MIN_SAFE_GAP``` away, the ego vehicle checks to see whether a lane change to the left or right is possible depending on the lane change clear flags and the current lane of the ego vehicle. Left lane changes are preferred over right lane changes to position the vehicle in a faster moving lane. Lanes are changed by incrementing or decrementing ```LANE_ID```, which is the current lane of the ego vehicle. This change places the downstream anchor points for the next trajectory in the new lane and creates a spline that transitions smoothly from the current lane to the new lane. 

Alternatively, if lane changes are not possible due to the close proximity of obstacles, the ego vehicle tracks the velocity of the obstacle in front of it until a safe gap is available in an adjacent lane to perform a lane change. 
