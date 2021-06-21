# AGV Library

by SS47816

### Status

<span style="color:green">*<Stable>*</span> <span style="color:green">*<Using lib>*</span> <span style="color:green">*<Good Code Style>*</span> <span style="color:green">*<Documented>*</span>

---

## Dependencies

* gcc (7.0 and above)
* Eigen (3.3)
* IPOPT (for MPC only)
* CppAD (for MPC only)

## Installation

1. Copy and paste the following files:

   ```bash
   agv/include/common/**
   agv/include/local_planner/**
   agv/include/mpc/**
   ```
   
2. Install CppAD and IPOPT:

   Go to `agv/include/mpc/` directory and run

   ```bash
   sudo ./install-ubuntu-MPC.sh
   ```

## Usage

* We encourage use/modify/expand the current library, please read through the library header files and find/modify/add the APIs you need.

* `local_planner_node.cpp`, `behaviour_planner.cpp`, etc. can be used as sample codes.

## APIs

### File System

#### Common Library

| Header File          | Contents                                                     |
| :------------------- | ------------------------------------------------------------ |
| frenet.h             | `FrenetState`, `FrenetPath` Class & Conversion to frenet frame |
| lane.h               | `Map`, `Path` Class & Find waypoint functions                |
| math_ultis.h         | math functions, unit conversions                             |
| motion_model.h       | vehicle motion model                                         |
| obstacle.h           | `CircleObstacle` Class                                       |
| polynomials.h        | polynomial functions                                         |
| quartic_polynomial.h | `QuarticPolynomial` Class                                    |
| quintic_polynomial.h | `QuinticPolynomial` Class                                    |
| spline.h             | `Spline`, `Spline2D` Class                                   |
| vehicle_state.h      | `VehicleState`, `ActuatorState` Class                        |
| vehicle.h            | `Vehicle` Class and its parameters                           |

####Local Planner Library

| Header File                         | Contents                        |
| ----------------------------------- | ------------------------------- |
| frenet_optimal_trajectory_planner.h | Implementation of the algorithm |

#### MPC Library

| Header File | Contents                            |
| ----------- | ----------------------------------- |
| MPC.h       | Implementation of the MPC algorithm |

## Contribution

* We use [Google C++ style](https://google.github.io/styleguide/cppguide.html) and we follow the code style of the [Apollo](https://github.com/ApolloAuto/apollo)
* Please continue expanding the library
