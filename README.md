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
If there is a front obstacle within the safe lookahead distance (`cp_inc`, set at 30 m), the ego car slows down without violating the acceleration and jerk constraints by decelerating at 5 m/s^2 (`ref_vel_delta` on line 10). It then checks whether it is safe to change lanes by first trying the left lane (state_PLCL) and then right lane (state_PLCR). This state also has a timer (`KL_count`) that biases the ego car to stay in the centre lane as much as possible, if it is driving for a longer time in the left or right lanes. This heuristic rule is practically useful to avoid traffic logjams since it allows the car the option of going either left or right in congested situations, instead of being stuck in an extreme lane. This logic is implemented in `state_KL()` (lines 247 to 294).

* Prepare Lane Change (state_PLCL and state_PLCR)

Once the planner has determined that a lane change is needed, it first transitions to the state_PLCL, which checks whether left lane is free from front and rear obstacle vehicles or not. It does this by using the state of the traffic obtained in `get_surrounding_vehicles()`(lines 48-234 in `vehicle.cpp`) and checking if there is sufficient gap to smoothly transition into the left lane. The gap is set to `cp_inc` for the front obstacle (lines 154, 184, 212) and 0.5*`cp_inc` for the rear obstacle (lines 170, 198, and 226). If there is sufficient gap, the lane number is changed to the target lane and the state changes to `state_LC`, otherwise the planner transitions back to state_KL after a certain number of tries, so that the right lane can be checked next. This logic is implemented in `state_PLCL()` (lines 296 to 339).

The identical algorithm is used for state_PLCR, except that the right lane safe transition conditions are checked. This logic is implmented in `state_PLCR()` (lines 341 to 384). If the safety conditions are not fulfilled in either the left or right PLC states, the state_KL attempts to reset the timers, reduces the current ego vehicle velocity to follow the car in front, and retries the lane change maneuver after the next sampling instant (lines 265 to 268).

* Change Lane (state_LC)

After the target lane has been set, this state monitors the lane change and determines when the maneuver is complete. The condition for completion is that the distance between the target d value and current d value is below a threshold. Once complete, the state transitions back to state_KL. The logic is implemented in `state_LC()` (lines 386 to 396).

## Path Generation

The car uses a perfect controller and will visit every (x,y) point it recieves in the waypoint list every .02 seconds. Hence, the main task of the trajectory generator (in file `Trajectory_Generator.cpp`) is to define a set of waypoints that obey the acceleration and jerk constraints while ensuring a smooth trajectory for the controller to follow. The logic for this is implemented in `generate_trajectory()`  (lines 159 to 258).

The approach used by me follows almost identically from the one described in the project Q&A video. It first smoothens out the transition from the previous trajectory by using either the current and previous ego car position or the left-over waypoints from the previous path returned by the simulator (lines 177 to 196). It uses 3 more control points that are `cp_inc` ahead of the last path waypoint and these 5 points determine the spline input (lines 202 to 213). All control points are then converted to the ego car coordinate frame and then given to the spline library to avoid numerical errors and make the calculation easier (lines 215 to 226).

To interpolate between the control points to generate 50 total points in the new trajectory, I approximate the target distance by a straight line and calculate the distance between adjacent waypoints, `x_inc`, using the hypotenuse approximation to the spline curve, as discussed in the video (lines 236 to 241). The x-coordinate of all these waypoints is used to calculate the y-coordinate from the spline, each waypoint (x,y) is converted back into global coordinates and then stored in the new waypoints list that will be given back to the simulator (lines 244 to 258). In this way, a new trajectory is generated, based on the target lane and ego car reference velocity defined by the lane change state machine.


## Performance
I tested the path planner over multiple runs of the simulator at 640x480 resolution. It successfully navigated the ego car through the highway track, avoiding obstacles and collisions, smoothly changing lanes when there was a slower moving car in front, and tried to keep close to 49.5 MPH whenever feasible. The bias towards the centre lane makes the car change lanes in advance to allow more chance of not getting stuck in slower moving traffic.

There are two limitations I observed with this approach. The first is that there are situations where the ego car will follow the front slower moving car for a while before it is able to change lanes, even if the extreme lane is free. This situation arises when there is a lot of traffic in two adjacent lanes and all the cars are slow moving. This is a consequence of the local approach I have used, which only looks at the current and next lanes to determine the optimal lane to drive in. A global cost-based approach could, in theory, account for the third lane also, and attempt a multi-lane change maneuver, if it found that the local traffic was preventing it from speeding up in a specified time. However, I was not able to implement adding cost functions to my current approach due to lack of time.

The second limitation I observed was for extreme cases, in terms of a front car suddenly braking excessively, or a collision involving multiple cars in front suddenly. I observed each of these situations only once during my entire testing but I feel I should mention it because my current approach does not have an emergency braking solution for such unsafe situations. However, for the vast majority of runs, the ego car was able to traverse the loop successfully multiple times, often going more than 12 miles without incident. A screenshot of one such successful run within the rubric stipulated time is saved in the image file `CarND_PathPlanning_Screenshot.jpg`.

## Future Work
* Extend the gap-based approach to allow multiple lane changes in a single maneuver.
* Implement emergency braking for rare events where accidents occur in the simulator.
* Use a cost function-based approach to allow the planner to make more global decisions about lane changing, instead of only looking at local traffic.
