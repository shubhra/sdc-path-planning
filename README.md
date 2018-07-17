# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    
---
## Rubric Points discussed

Simulator Video:
- You can find one loop of the car around the simulator track [here](https://youtu.be/QhI7Mk5-zxQ)

The car doesn't drive faster than the speed limit
- No speed limit warning message show up during the run. The max speed is set to 49.5mph just shy of a 50 mph limit to avoid crossing this at any time.

The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3
- Max jerk warning message does not show during the simulator run. The car speeds up and slows down at roughly 5m/s to avoid the reaching acceleration

The car must not come into contact with any of the other cars on the road
- No collisions were reported and the car manouvers safely

The car stays in its lane, except for the time between changing lanes.
- The car stays in its lane most of the time except when it changes lane because of traffic or to return to the center lane from the leftmost lane

The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes
- The car attempts to change lanes whenever there is a slow car ahead of it. It checkes if it is safe to change lanes (no other cars in the left or right lane too close ahead (~40m) or too close behind (~5m)) or when it is safe to return the center lane from the leftmost lane

The car is able to smoothly change lanes when it makes sense to do so
 - it smoothly changes lanes when it sees an opportunity to do so

Reflection

The code consist of three major parts:

Prediction Phase

This part of the code checks the cars surroudnings using telemetry and sensor fusion data to find cars ahead of, in the left  or right lane. It almo makes judgement regarding whether-
- Other cars are too close ahead of us (less than 30m ahead)
- Other cars are too close in the left/right lane ahead/behind us for a comfortable/sensible lane shift (less than 40m ahead and less than 5m behind). Also keeps track whether is it better to make a left or a right lane shift in case both are open.
 

Behavior Phase

This part decides what steps to take in the following scenarios from the prediction phase:

If a car is ahead of us in our lane:
- Slow down slowly to match that cars speed
- Look for an opportunity to shift lane to the left or right whichever is open
If no car is ahead of us:
- increase speed slowly to reach speed limit
- if we are in the leftmost lane then shift over one lane to the right (if open)

Trajectory Phase

This part calculates the trajectory to give to the simuator. 

First, the last two points of the previous trajectory (or the current position of the car during the initial step) are used along with 3 more points at a further distance ahead of us (at 30m, 60m and 90m) to initialize the spline. These 5 points are transformed to the cars coordinate system for ease of calculation. For continuity of trajectory, previous points that were acted upon in the previous pass are kept. New points are also added to the trajectory by evaluating the spline at 30m and transforming the output coordinates to map coordinates.

---
