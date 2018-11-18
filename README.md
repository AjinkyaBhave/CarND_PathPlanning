# CarND-Path-Planning-Project

## Introduction

### Project Goal
The goal of the path planning project is to design a path planner that is able to create smooth, safe paths for the virtual car to follow along a three lane highway with traffic. A successful path planner will be able to keep the car inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

### Project Rubrics
1. The car is able to drive at least 4.32 miles without incident.
2. The car drives according to the speed limit.
3. The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
4. Car does not have collisions. 
5. The car stays in its lane, except for the time between changing lanes. 
6. The car is able to change lanes smoothly when it makes sense to do so.

The following sections will describe how each of the project rubrics have been addressed successfully.

## Code Structure
The code contains two main classes where the path planning behaviour and trajectory generation have been implemented. 

`main.cpp` contains the original Udacity code to connect to the simulator and obtain environment and vehicle data at each sample time. It also instantiates and calls the Vehicle and Trajectory_Generator classes and associated functions.

`vehicle.cpp` contains the Vehicle class which maintains the state of the ego vehicle (position in x,y and Frenet coordinates, along with speed and state of obstacle vehicles in neighbouring lanes). It also contains the behaviour planner algorithm which chooses which state to transition to next, based on current environment and ego car state. 

`trajectory_generator.cpp` contains the Trajectory_Generator class which reads the waypoint map file, implements all the waypoint coordinate transformation functions originally contained in `main.cpp` and generates the trajectory for the ego car to follow.

The code has been commented in detail, especially the variables, functions, and the major parts of the algorithms.


## Behaviour Planner
The approach I have used is a state-based one, where there are 4 possible states the ego car can be in. The state machine is implemented in `vehicle.cpp` in the appropriately named functions. The states are:
* Keep Lane (state_KL)

This is the default state of the ego car. When the simulator starts, it is assumed that the car starts in this state in the centre lane. This state checks whether there is any obstacle vehicle in the current lane. If not, the car tries to drive at the maximum velocity, which is set at 49.5 MPH.
If there is a front obstacle within the safe lookahead distance (`cp_inc` set at 30 m), the ego car slows down and checks whether it is safe to change lanes by first trying the left lane (state_PLCL) and then right lane (state_PLCR). This state also has a timer (`KL_count`) that biases the ego car to stay in the centre lane as much as possible, if it is driving for a longer time in the left or right lanes. This heuristic rule is practically useful to avoid traffic logjams since it allows the car the option of going either left or right in congested situations, instead of being stuck in an extreme lane. This logic is implemented in `state_KL()` (lines 247 to 294).

* Prepare Lane Change (state_PLCL and state_PLCR)

Once the planner has determined that a lane change is needed, it first transitions to the state_PLCL, which checks whether left lane is free from front and rear obstacle vehicles or not. It does this by using the state of the traffic obtained in `get_surrounding_vehicles()`(lines 48-234 in `vehicle.cpp`) and checking if there is sufficient gap to smoothly transition into the left lane. The gap is set to `cp_inc` for the front obstacle (lines 154, 184, 212) and 0.5*`cp_inc` for the rear obstacle (lines 170, 198, and 226). If there is sufficient gap, the lane number is changed to the target lane and the state changes to `state_LC`, otherwise the planner transitions back to state_KL after a certain number of tries, so that the right lane can be checked next. This logic is implemented in `state_PLCL()` (lines 296 to 339).

The identical algorithm is used for state_PLCR, except that the right lane safe transition conditions are checked. This logic is implmented in `state_PLCR()` (lines 341 to 384). If the safety conditions are not fulfilled in either the left or right PLC states, the state_KL attempts to reset the timers, reduces the current ego vehicle velocity to follow the car in front, and retries the lane change maneuver after the next sampling instant (lines 265 to 268).

* Change Lane (state_LC)

After the target lane has been set, this state monitors the lane change and determines when the maneuver is complete. The condition for completion is that the distance between the target d value and current d value is below a threshold. Once complete, the state transitions back to state_KL. The logic is implemented in `state_LC()` (lines 386 to 396).

## Path Generation

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.
cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)

## Future Work
* Use a cost function-based approach to allow the planner to make more global decisions about lane changing, instead of only looking at local traffic.
* Extend the gap-based approach to allow multiple lane changes in a single maneuver.
* Implement emergency braking for rare events where accidents occur in the simulator.
