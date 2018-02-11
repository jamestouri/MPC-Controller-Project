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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

## The Model 

### Model State
The model state will be represented based on the vector of {px, py, psi, v, cte, epsi}, where the data for these points come from the simulator that the car will be running in.  The simulator provides this data on the center of the road as waypoints. 

### Model Equations and Cost
The car is run upon these 5 equations for the Model State:
```
x1 = (x0 + v0 * CppAD::cos(psi0) * dt);
y1 = (y0 + v0 * CppAD::sin(psi0) * dt);
psi1 = (psi0 + v0 * delta0 / Lf * dt);
v1 = (v0 + a0 * dt);
cte1 = ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
The equations are also needed to provide an estimated state of a value of 100ms in the future. 

The model cost was given through Udacity and used it for error states of cte, epsi, and velocity.

Cost Model:

```
   // CTE, orientation error and ref-speed error
        for (int i = 0; i < N; i++) {
            fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
            fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
            fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
        }
        
        // Minimize the use of actuators.
        for (int i = 0; i < N - 1; i++) {
            fg[0] += CppAD::pow(vars[delta_start + i], 2);
            fg[0] += CppAD::pow(vars[a_start + i], 2);
        }
        
        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < N - 2; i++) {
            fg[0] += 5000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
        }
```

### Model Speed, Time Step Length an d Elasped Duration

I chose the model speed at a high of 25.  Working at higher speeds had the car drift off at the bridge, and sometimes at the first dirt road with clear unstability. 

The N value is appointed at 10 and dt at 0.05. Various values were tested and these resulted in the best values at 20:1 ratio.

### MPC Preprocessing

The polynomial was created to fit into the waypoints and implements the transform matrix. The result of the polynomial fitting are coefficients which will serve to calculate the cross-track error CTE.

### Latency 

Being in a simulated program, previous projects didn't have a need for a dt value, however with an estimation of the vehicle's state of the 100ms into the future, the vehicle's stability changed based on the values given. 


![](https://github.com/jamestouri/MPC-Controller-Project/blob/master/Screen%20Shot%202018-02-11%20at%2011.46.56%20AM.png)






