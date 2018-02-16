# Model Predictive Control (MPC)
MPC is an advanced process control method that makes use of the dynamic model of the system to optimize future control paramters. In this case the MPC is used to control the *throttle value*, and *steering angle* of a self-driving car. The MPC controller takes advantage of the Kinematic vehicle model for achieving this task.

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

---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

---
# MPC implementation
## The Model
The model used here is a Kinematic model (compared to a dynamic model, kinematic model is simpler because it ignores parameters like gravity, tire forces and mass). Kinematic model is chosen for its simplicity and effectiveness.  
#### States
The states are vehicles position in **x** and **y** coordinate, orientation/heading angle **(psi)**, velocity of the vehicle **(v)**, cross-track error **(cte)** ie. the lateral distance of the vehicle from the planned trajectory, and orrientation error **(e_psi)** ie. the difference of actual vehicle orientation and trajectory orientation.  
_State vector --> [x,y,ψ,v,cte,eψ]_
#### Controls
The controls are the actuators that can be adjusted to obtain the desires trajectory. For a car actuators are steering angle, throttle and brakes. Here we consider throttle and brakes as one parameter (as braking is negative acceleration). So there are two cotrol variables the MPC will be predicting. Acceleration **(a)** and steering angle **(delta)**.  
_Control vector --> [δ,a]_
#### Update equations
![global kinematic model](https://github.com/askmuhsin/model-predictive-cotroller/blob/master/images/global_kinematic_model.png)  
![global kinematic model](https://github.com/askmuhsin/model-predictive-cotroller/blob/master/images/global_kinematic_model_cte.png)  
![global kinematic model](https://github.com/askmuhsin/model-predictive-cotroller/blob/master/images/global_kinematic_model_epsi.png)  


## Timestep Length and Elapsed Duration (N & dt)
Two of the most important hyperparameters in tuning the MPC are: **N** (timestep length) and **dt** (elapsed duration between timesteps) values. The objective is to choose a value so that the Cross track error is optimal (zero) while not being computationally costly.  
The values chosen in this project is **N=10 and dt=0.1**, thus obtaining a trajectory duration of 1s. _(T=N*dt)_   
These values were picked from the Udacity support forums. Different values were later tried out like N=25 & dt=0.05 and some other values. But _N=10 and dt=0.1_ seems to behave the best.  

## MPC Preprocessing
The waypoints were preprocessed. The x, y coordinates and orientation angle of the vehicle state were shifted to origin in respect to vehicles perspective. This was done to simplify the calculations. (see main.cpp line 101:109)

## Model Predictive Control with Latency
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.


---
# Result
Code compiles without errors with `cmake` and `make`.
The vehicle drives around the track without leaving the drivable portion. [Video link](https://youtu.be/tY872fwnqFE).
![Result]()

