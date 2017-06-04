# CarND-Controls-PID

This project is to implement a PID controller to drive a car following a given path in the [simulator](https://github.com/udacity/self-driving-car-sim/releases).

### `PID Class`

The PID controller is implemented by a `PID Class`, which needs to be specified the P, I, D coefficients:

- `Kp` - proportional coefficient
- `Ki` - integral coefficient
- `Kd` - differential coefficient

This class has two methods: `UpdateError()` and `TotalError()`. 

`UpdateError()` updates the errors of the three coefficients, which are listed as follows:

```c++
  d_error = cte - p_error; // The differential error is updated according the (current_cte - previous_cte)
  p_error = cte; // The proportional error is updated according to the cte
  i_error += cte; // The integral error is updated according to the accumulated cte
```

`TotalError()` calculates the weighted total error:

```c++
total_error = Kp*p_error + Ki*i_error + Kd*d_error;
```

### P, I, D Tuning

Those three parameters were tuned manually by trial and error based on the following principles.

- `Kp` cannot be too high to avoid overshooting and oscillation.
- `Kd` is to consider the rate of change of error, which slows down the proportional control but overcomes the overshooting and oscillation issues.
- `Ki` is to consider the accumulated error to fix the systematic bias, which should be a small value.

Keeping those in mind, the three parameters are chosen after some trial and error:

```c++
double Kp_steer = 0.1;
double Ki_steer = 0.005;
double Kd_steer = 5.0;
```

By using the above parameters, the PID controller can drive the car and finish the track. Although the car still has some oscillations, but it does not run off the road. 

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

  * e.g. `./term2_sim.x86_64` to run the simulator in Linux
  * need to set `Allow executing file as program` for this file Linux

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## TODOs

- A PID controller of the speed is added but its parameters need to be fine tuned.

