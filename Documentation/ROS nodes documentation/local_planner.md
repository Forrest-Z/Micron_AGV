# Local Planner

by SS47816

### Status

<span style="color:green">*<Stable>*</span> <span style="color:green">*<Using lib>*</span> <span style="color:green">*<Using cfg>*</span> <span style="color:green">*<Good Code Style>*</span> <span style="color:green">*<Documented>*</span>

---

## Dependencies

* gcc (7.0 and above)
* Eigen (3.3)

## Installation

1. Copy and paste the following files:

   ```bash
   agv/include/common/**
   agv/include/local_planner/**
   agv/src/local_planner_node.cpp
   agv/launch/local_planner_node.launch
   agv/cfg/local_planner.cfg
   ```

## Usage

```bash
roslaunch agv local_planner_node.launch
```

## Code Explained

### 1. Algorithm

Please refer to this paper ["Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame"](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame) 

### 2. Topics

| Subscribe to       |                                                           |
| :----------------- | --------------------------------------------------------- |
| /odometry/filtered | Odometry                                                  |
| /obstacles         | 3D Lidar obstacles                                        |
| /obstacles2        | 2D Lidar obstacles                                        |
| /lane_info_topic   | Lanes                                                     |
| /cmd_vel_out       | Final control output (for path regeneration when braking) |
| /behaviour         | Commands from behaviour planner (currently not used)      |

| **Publish to**          |                                  |
| ----------------------- | -------------------------------- |
| /output_path            | The current executing trajectory |
| /next_path              | The next best trajectory         |
| /ref_path               | The reference spline             |
| /desired_steering_angle | Stanley output                   |
| /turn_signal            | For LED and GUI                  |

### 2. Parameters

1. Run:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

2. Select `local_planner_node` and you will see the following parameters:

| Hyperparameters      | Default | Comment                                                      |
| -------------------- | ------- | ------------------------------------------------------------ |
| output_path_max_size | 10      | the greater the value is, the longer the output path will be. |
| output_path_min_size | 7       | the min path length need to satisfy the requirement of the Stanley |

| Sampling Parameters (lateral) | Default | Comment                  |
| ----------------------------- | ------- | ------------------------ |
| left_lane_width               | 3.0     |                          |
| right_lane_width              | 3.0     |                          |
| center_offset                 | -0.3    | right being negative     |
| delta_width                   | 0.3     | sampling width increment |

| Sampling Parameters (longitudinal) | Default | Comment                |
| ---------------------------------- | ------- | ---------------------- |
| max_t                              | 10      | max sample time        |
| min_t                              | 5       | min sample time        |
| delta_t                            | 2.5     | target time increment  |
| tick_t                             | 0.3     | time step for sampling |
| target_speed                       | 3.0     | desired speed          |
| delta_speed                        | 1.0     | speed increment        |
| num_speed_sample                   | 0       | not varying speed      |

| Cost Weights   | Default | Comment                                |
| -------------- | ------- | -------------------------------------- |
| k_jerk         | 1.0     | total squared jerk                     |
| k_time         | 1.0     | sampling time                          |
| k_diff         | 1.0     | difference in speed and lateral offset |
| k_lateral      | 1.0     | lateral overall weight                 |
| k_longitudinal | 1.0     | longitudinal overall weight            |
| k_obstacle     | 0.5     | distance to obstacle                   |

| Safety Constraints | Default | Comment                             |
| ------------------ | ------- | ----------------------------------- |
| hard_safety_margin | 0.3     | in addition to vehicle's width      |
| soft_safety_margin | 1.0     | in addition to `hard_safety_margin` |

| Other Parameters     | Default | Comment                      |
| -------------------- | ------- | ---------------------------- |
| stanley_overall_gain | 0.6     | overall gain                 |
| track_error_gain     | 0.2     | cross-track error gain       |
| turn_yaw_thresh      | 0.785   | for turn signal (45 degrees) |

### 3. Implementation

* Please refer to my FYP report for detailed implementations

### 4. Performance

* Time Complexity: O(num_width * num_time * num_speed * (`t`/`tick_t`) * num_obstacles)
* Space Complexity: O(num_width * num_time * num_speed * (`t`/`tick_t`))
* 20% CPU usage (single core used) inside workshop space (less obstacles) at 10Hz
* 60% CPU usage (single core used) average at 10Hz

## Future Work

1. Improve the obstacle results in the perception layer (rectangle obstacles)
2. Include `/obstacles_3` topic into consideration (not recommended)
3. Consider dynamic obstacles in the `checkCollision()` function in `frenet_optimal_trajectory_planner.cc`
4. Parameterize `left_lane_width` and `right_lane_width` in `local_planner_node.cpp`, use actual lane width instead of fixed values.
5. Receive commands from the behaviour planner (can modify the `msg/behaviour.msg`)
6. Squzze its performance:
   * Increase the planning frequency (to 15Hz?)
   * Increase the number of samplings
   * Decrease sampling increments to produce finer samples
7. Change the calculation of the `cross_track_error` and `heading_error` in the `calculateSteeringAngle()`  from the current geometrical method to the polynomial method used in `mpc_controller_node.cpp` (with the help of `include/common/polynomials.h`)
8. Separate the Stanley algorithm into a separate node (can use a similar implementation as the `mpc_controller_node.cpp` ) 
9. Improve the code style and continue integrating libraries



