# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).



---

Writeup
---

**Vehicle Detection Project**

---

# Compilation

Compilation works without errors and warnings like usual:

* cmake ..
* cmake --build .
* ./mpc

# Implementation

## The Model

The kinematic model consists of a the vehicle position (in x and y coordinates, orientation angle psi). Moreover, is a CTE (cross-track error) as well as a psi error (epsi) calculated to reduce keep the car on track while reducing these to zero.
For this, a certain acceleration and steering angle are required as output from calculations based on the previous measurements.

The model basically contains the following elements:

* ptsx (x position of waypoints)
* ptsy (y position of waypoints)
* px (current x position of the vehicle)
* py (current y position of the vehicle)
* psi (orientation of the vehicle)
* v (velocity of the vehicle)
* delta (steering angle)
* a (throttle)

## Timestep Length and Elapsed Duration (N & dt)

Based on the reference trajectory from the polynomial fit of waypoints, the MPC algorithm calculates the waypoints as predictions for a defined prediction horizon T.

T = N * dt

This means the total prediction time is split into the number of waypoints N with the equal distance of dt.

The combination of N and dt is crucial for the performance of the algorithm. Is N too high, the vehicle reacts too late and takes sharp turns. Is N too low, the prediction horizon is not sufficient and the car can easily miss the lane.
dt determines the time step length between the waypoints. Is it too large, the vehicle cannot stir in the right direction before ending up in the wrong place. Is it too small, the calculation time is longer than the time step length.

Moreover, the combination has also an effect: 
* N * dt too large, the calculation needs too long. 
* N * dt too small, the prediction time is not sufficient and vehicle starts oscillating.

The chosen values N = 12 and dt = 0.15 seems to be the perfect tradeoff for the reference velocity as well as for this use case.

## Polynomial Fitting and MPC Preprocessing

First, the global coordinates of the simulator needs to be transformed into vehicle coordinates. This is done in the MPC.cpp in the function Glob2CarX and Glob2CarY with a 2D vector transformation equation.

With this done, a thrird-degree polyfit algorithm can be used to fit the transformed waypoints into a discrete curve.
Due to the fact that all these calculations are done in the vehicle's perspective, the component px, py and psi can be set to zero because the vehicle is in the center of the coordination system.  
With the provided polyeval function the CTE can be calculated, so that the error from the reference trajectory can be counted.
Furthermore, the psi error (epsi) is calculated from the derivative of polynomial fit. It is the negative arctan of the 2. coefficient of the polyfit.

## Model Predictive Control with Latency

The model performs pretty good for low velocities without taking the latency into account. But with higher velocities the vehicle behaviour results in an undeterminstic one. 

To implement a solution for this problem, a function "StateWithLatency" is added that takes the old state of the vehicle and assumes constant velocity and steering angle to calculate the new state after the given latency (100 ms) later with a usual kinematic approach.

With this new state, the new steering angle and throttle is calculated and the latency problem is solved.

# Simulation

The car doesn't leave the road. The result is very satisfying because it drives so accurate in the lane and almost everytime stays in the middle of the lane.
