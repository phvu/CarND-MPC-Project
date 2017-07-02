# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Project Rubic

### Compilation

The project can be compiled and executed with:

```bash
mkdir -p build && cd build && cmake .. && make
./mpc
```

### Implementation

#### The Model

The model is a vector with 6 elements: `x, y, psi, v, cte, epsi`, which are the x coordinate, y coordinate, orientation,
velocity, cross-tract error and the orientation error of the vehicle.

In this project, I used the x and y coordinates in the car's coordinate system, with the x axis points in the 
direction where the car is heading to, and the y axis points to the left.

The actuators include the _steering angle_ `delta` and _acceleration_ `alpha`. At time `t`, given the actuators,
the new model vector can be updated using the following equations:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where:

- `dt` is the interval between consecutive time-steps `t-1` and `t`
- `Lf` is the distance between the front of the vehicle and its center of gravity
- `f(x)` is the motion model, normally is a fitted 3rd-order polynomial.
- `psides` is the desired orientation, computed as `arctan(f'(x_t))`

This model is used in `MPC.cpp:72:110` to set up the objective function to be optimized.

#### Timestep Length and Elapsed Duration

The timestep length is chosen as `N=10`. This is the number of steps to predict ahead in the future.
Larger values of `N` will lead to more complicated optimization problem because the objective function 
and constraints have to take into account more terms (and increase computation time), also our model is 
not perfect and will likely degrade if we look too far ahead.
Smaller values of `N` may be insufficient to model difficult scenario. I tried to set `N` between 8 and 20,
and `10` turns out to be more stable.

The Elapsed duration `dt` measures the duration between consecutive timesteps. Since the car is modelled with 
a delay of 0.1 seconds, I chose `dt=0.1` for convenience. When `dt < 0.1` and I take the first actuator, the car
tend to drive backward. Larger value of `dt` might create bigger mistake and the car might not be able to recover.

#### Polynomial Fitting and MPC Preprocessing

The waypoints are first converted into the car's coordinate system, then fitted with a 3rd-order polynomial. This is 
done between lines `124-134` of `main.cpp`.

#### Model Predictive Control with Latency

Since we have a control latency of `0.1 second`, we need to predict the state of the car at 0.1 second after the 
_current_ moment. This is done using the update equations above (with `x=y=psi=0, dt=0.1`), 
and implemented between lines `139-151` of `main.cpp`.

This step is crucial in order to make the algorithm works. We then feed this state vector into MPC optimization algorithm.

The optimization will give an _optimial_ estimation for the actuators `delta` and `alpha` for `N-1` timesteps ahead.
The final actuators are chosen to be the average of the first 3 estimated values. This is to make sure we have 
a stable and smooth actuation. It is implemented between lines `225-233` of `MPC.cpp`.

The objective function turned out to be quite tricky. I tried to throw in some of my ideas (the steeper the car
is turning, the slower it should go, penalizing the desired velocity if the car is turning steep, etc...) but those
lead to unstable scenarios.

I tried to put stricter constrained on the variables. This is done on lines `154-167` of `MPC.cpp`. Among those constraints,
probably only constraints on the _steering angle_ and the _acceleration_ are crucial. Contraints on the `x` and `y` coordinates
are a bit superficial and probably only apply on the simulator (and shouldn't be that strict in practice).

### Simulation

The algorithm can be executed with `./mpc`

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
