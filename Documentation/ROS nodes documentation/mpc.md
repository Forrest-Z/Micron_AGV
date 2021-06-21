# Model Predictive Control

by SS47816

### Status

<span style="color:red">*<Unstable>*</span> <span style="color:orange">*<Partially Using lib>*</span> <span style="color:red">*<Not Using cfg>*</span> <span style="color:orange">*<Partially Good Code Style>*</span> <span style="color:green">*<Documented>*</span>

---

## Dependencies

* gcc (7.0 and above)
* Eigen (3.3)
* IPOPT
* CppAD

## Installation

1. Copy and paste the following files:

   ```bash
   agv/include/common/**
   agv/include/mpc/**
   agv/src/local_planner_node.cpp
   agv/launch/local_planner_node.launch
   agv/cfg/local_planner.cfg
   ```

2. Install CppAD and IPOPT:

   Go to `agv/include/mpc/` directory and run

   ```bash
   sudo ./install-ubuntu-MPC.sh
   ```

## Usage

1. Settings
   * If only control the steering:
     1. In `mpc_controller_node.launch`, change the  `steering_angle_topic` to: `/desired_steering_angle`
     2. In `local_planner_node.launch`, change the `steering_angle_topic` to: `/dummy_steering_angle`
   * If control both the steering and the speed:
     1. change the  `control_output_topic` to: `/nav_cmd_vel`
2. Run

```bash
roslaunch agv mpc_controller_node.launch
```

## Code Explained

### 1. Algorithm

Please refer to this [Matlab MPC tutorial](https://www.youtube.com/playlist?list=PLn8PRpmsu08ozoeoXgxPSBKLyd4YEHww8)

### 2. Topics

| Subscribe to       |                                                           |
| :----------------- | --------------------------------------------------------- |
| /odometry/filtered | Odometry                                                  |
| /output_path       | Output trajectory from the local planner                  |
| /behaviour_cmd_vel | Receive the reference velocity from the behaviour planner |

| **Publish to**       |                                                              |
| -------------------- | ------------------------------------------------------------ |
| steering_angle_topic | Publish only the steering angle                              |
| control_output_topic | Publish both the steering angle and the velocity (dac voltage) |

### 2. Parameters

Inside `mpc_controller_node.launch`:

| Hyperparameters    | Default | Comment      |
| ------------------ | ------- | ------------ |
| mpc_frequency      | 10      |              |
| prediction_horizon | 25      | Time steps   |
| control_horizon    | 5       | Not used yet |
| sample_time        | 0.2     | delta t      |

| Path Related Parameters (Should be moved to .cfg) | Default | Comment                   |
| ------------------------------------------------- | ------- | ------------------------- |
| max_path_size                                     | 15      | Maximum allowed path size |
| min_path_size                                     | 5       | Minimum allowed path size |
| order_of_poly                                     | 3       | For interpolating path    |

| Vehicle Model Params (Should be replaced with library) | Default | Comment                         |
| ------------------------------------------------------ | ------- | ------------------------------- |
| baselink_to_front_axle_length                          | 2.2     | baselink to front axle          |
| front_to_cog_length                                    | 1.1     | front axle to centre of gravity |

| Cost Weights (Should be moved to .cfg) | Default | Comment                                           |
| -------------------------------------- | ------- | ------------------------------------------------- |
| cross_track_error_weight               | 1.0     | Minimise the cross track error                    |
| orientation_error_weight               | 1.0     | Minimise the heading error                        |
| speed_diff_error_weight                | 1.0     | Encourage driving accroding to reference velocity |
| steering_usage_weight                  | 10.0    | Penalise the use of steering wheel                |
| acceleration_usage_weight              | 1.0     | Penalise the use of throttle                      |
| steering_smoothness_weight             | 50.0    | Penalise drastic change in steering angle         |
| acceleration_smoothness_weight         | 1.0     | Penalise drastic change in accelerator            |

| Safety Constraints (Should be replaced with library) | Default | Comment |
| ---------------------------------------------------- | ------- | ------- |
| max_steering_angle                                   | 0.336   |         |
| max_acceleration                                     | 1.0     |         |
| max_deceleration                                     | -3.0    |         |

### 3. Implementation

* Please refer to my FYP report for detailed implementations

### 4. Performance

* The vehicle tended to overshoot and oscillate badly
* 100% CPU usage (single core used) at 10Hz

## Future Work

1. Switch to new libraries

   * Use the constraints set in `inlcude/common/vehicle.h`
   * Rewrite some of the functions using the available APIs
   * Move some `mpc_controller_node` functions to the `common` lib, such as `getFrontAxlePose()`, `getPolynomialCoeffs()`, `calcCrossTrackError()`, `calcOrientationError()`
   * Create a `MPC::Setting` Class in `include/mpc/MPC.h`for passing parameters from `mpc_controller_node` to `MPC`

2. Create a `mpc_controller.cfg` file and move the important parameters to it

3. Tune the weights

4. Improve its performance: 

   * Refine the vehicle motion model used in the MPC

   * To replace the hard-coded slowdown when steering angle is too big in the `behaviour_planner_node.cpp`:

     a) Add curvature of the trajectory as a new target cost function (use `next_path` if the `output_path` is too short). 

     b) Add the total squared jerk as a new target cost function (use `next_path` if the `output_path` is too short). 

5. Improve its performance: (aim to reach 10Hz of control frequency with 0.1s time step)

   * Add a control horizon to the algorithm (to reduce the computational load)
   * Reduce the valid value range for each parameter before sent to the optimiser

6. Improve the code style and continue integrating libraries



