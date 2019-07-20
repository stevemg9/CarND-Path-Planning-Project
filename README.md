# Path Planning Project Report
Stephen Giardinelli

## Path Planning Approach
For this project, I followed the general guidelines provided in the Project Q&A.  I found that the general strategies discussed there worked fairly well and just needed some improvements and fine tuning.  The bulk of my time was spent dealing with lane chgane logic.  My entire path planner was implemented using decision trees.  Though more complicated methods were described in the lectures, I found that they were not required in order to solve the problem provided.

## Driving Along A Lane
I started the project by trying to get the vehicle to drive along a lane.  In order to do this I mostly followed the approach provided in the Project Q&A.  At all times, the vehicle has a target lane and a target velocity.  The path is created by generating a spline which initially starts with two points - the inital point is generated behind the vehicle and second point is the vehcile's position.  This is needed to "point" the spline alone the lane.  The rest of the spline is created with 3 points that are projected 30m, 60m, and 90m along the target lane.  This points are projected using frenet coordinates and then the global cartesian coordinated are provided to spline generator.  One the vehicle is moving and a previous path is returned the the last two "previous path" points are used for the inital two spline points.  The the same three points are project out along the target lane.

I played around a decent amount with the spline poitns.  I tried using upt to 6 spline points with differnet spacing.  I found that the most important point is the first projected point as this will greatly affect how gradually or abruptly you change lanes.  30m seemed to be the sweet spot.  Also, I noticed that adding spline points did not help smooth out the spine, in fact it caused the spline to oscillate too much and sometimes the vehcile would bounce around laterally in the lane.

## Accelerating and Decelerating
The inital method proiveded in the Q&A did not seem optimal to me.  Instead I developed an approach that controlled the acceleration by setting the changing the gap in between waypoints.  The `target_gap` was calculated by converting the `target_speed` to m/s and then multiplying by 0.02s (20ms) to get the distance needed to move between each frame:
```c++
double target_gap = (target_speed / 2.24) * 0.02; 
```
Knowing the target distance between waypoints I handled acceration by increasing or decreasing the gap at every waypoint.  The value that I increased or decreased by `delta_gap` was calculated by finding the difference between the `current_gap` and `target_gap` and then normalizing that value and multiplying by a delta value that corresponeded to a maximum accereration of 8.75m/s^2.
```c++
double delta_gap = fabs(target_gap - current_gap) / 0.447 * 0.0035;
```
0.447 is the maximum difference in gap that we should see - like when the vehicle has a current velocity of 0 but a target velocity of 50mph.  This value normalizes the differnce value between 0 and 1.  This value is then multiplied by maximum `delta_gap` of 0.0035 (8.75m/s^2).  The result is a `delta_gap` that is lineraly proportional to the difference between `current_speed` and `target_speed` and ranges from 0 (0m/s^2) to 0.0035 (8.75m/s^2).

This method gave a very smooth acceleration which greatly helped limit the jerk while driving around the track.  The calculation, however, is not exact becase the gap is only calculated along the direction that the car is travelling and does not take into account the curvature of the car.

## Avoiding Collisions and Changing Lanes
I won't go through all of the logic for this, but I used the same ideas discussed in the Q&A and expanded on them.  To avoid collision I iterated through all of the snesor fusion data and:
* Is the vehicle in my `target_lane`?
* If so, is the vehicle less than 25m in front of me?
* If so, there could be 2 multiple vehicles in my lane, less than 25m in front of me, is this the closest one?
* If so, is this vehicle travelling slower than my `target_velocity`?
* If so, execute left lane change if possible
* If left lane change is not possible, execute right lane change if possible.
* If right lane change is not possible, slow down by changing my `target_speed` to the speed of the detected vehicle - 5mph.

I made a separate funciton to evaluate if a lane change is possible `canChangeLanes()`.  This funciton takes arguments for the `target_lane`, the `s_coordinate` at the end of the previous path, and the `sensor_fusion` data passed by reference.  It returns `true` if a lane change is possible and `false` if a lane change is not possible.

The logic of `canChangeLanes()` is very similar to detecting collisions, except it makes sure the target lane is clear for 10m in front of the end of the previous path and 18m behind the end of the previous path.

The tricky part of this part of the code was dealing with edge cases where the either the Ego vehicle or detected vehicles crossed the maximum s value of the track loop while the other did not.  I handled these cases by summing the values and taking the modulus with `fmod()`.

## Summary
After testing my path planning code numerous times, I have made many successful laps without incident.  I have a feeling there still might be some edge cases where acceleration is exceeded but that can be dealt with by simple lowering the target velocity or the maximum possible `delta_gap`.

The logic for changing lanes could be improved by using a cost funciton to evaluate which would be the most beneficial lane to change into instead of always attempting a left lane change before a right.  Overall I feel that my simple solution works just fine as the times when the Ego vehicle gets stuck behine a car, there is usually a large cluster of cars that even a more complex planning algorithm would not be able to get around.


# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

