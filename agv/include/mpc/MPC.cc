/* MPC.cc

  Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

  Model Predictive Control Class Implementation
*/

#include "MPC.h"

namespace agv
{
namespace mpc
{

MPC::MPC()
{
  N = 25;
  dt = 0.2;

  Lf = 1.1;
  max_steering_angle = 0.336;
  max_acceleration = 1;
  max_deceleration = -3;

  weights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  v_start = psi_start + N;
  cte_start = v_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  a_start = delta_start + N - 1;
};

MPC::MPC(const std::vector<double> &params, const std::vector<double> &weights, const std::vector<double> &constraints)
{
  // Params
  this->N = params.at(0);
  this->M = params.at(1);
  this->dt = params.at(2);
  this->Lf = params.at(3);

  // Weights
  this->weights = weights;

  // Constraints
  this->max_steering_angle = constraints.at(0);
  this->max_acceleration = constraints.at(1);
  this->max_deceleration = constraints.at(2);


  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  v_start = psi_start + N;
  cte_start = v_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  a_start = delta_start + N - 1;
}

// MPC destructor
MPC::~MPC() {}

std::vector<double> MPC::Solve(const Eigen::VectorXd &x0, const Eigen::VectorXd &coeffs, double ref_v)
{
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double v = x0[3];
  double cte = x0[4];
  double epsi = x0[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i)
  {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; ++i)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -20 and 20
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; ++i)
  {
    vars_lowerbound[i] = -max_steering_angle;
    vars_upperbound[i] = max_steering_angle;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; ++i)
  {
    vars_lowerbound[i] = max_deceleration;
    vars_upperbound[i] = max_acceleration;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // Object that computes objective and constraints
  std::vector<size_t> sizes =
  {
    N,
    x_start,
    y_start,
    psi_start,
    v_start,
    cte_start,
    epsi_start,
    delta_start,
    a_start
  };

  FG_eval fg_eval(coeffs, weights, sizes, dt, Lf, ref_v);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  //
  // Check some of the solution values
  //
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  return {solution.x[x_start + 1], solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start], solution.x[a_start]};
}

// FG_eval class constructor
MPC::FG_eval::FG_eval(Eigen::VectorXd coeffs, std::vector<double> weights, std::vector<size_t> sizes, double dt, double Lf, double ref_v)
{
  this->coeffs = coeffs;
  this->weights = weights;

  this->N = sizes.at(0);
  this->x_start = sizes.at(1);
  this->y_start = sizes.at(2);
  this->psi_start = sizes.at(3);
  this->v_start = sizes.at(4);
  this->cte_start = sizes.at(5);
  this->epsi_start = sizes.at(6);
  this->delta_start = sizes.at(7);
  this->a_start = sizes.at(8);

  this->dt = dt;
  this->Lf = Lf;
  this->ref_v = ref_v;
}

// FG_eval class composite function defination
void MPC::FG_eval::operator()(ADvector &fg, const ADvector &vars)
{
  // The cost is stored is the first element of `fg`.
  // Any additions to the cost should be added to `fg[0]`.
  fg[0] = 0;

  // Reference State Cost
  for (int t = 0; t < N; ++t)
  {
    fg[0] += weights[0] * CppAD::pow(vars[cte_start + t], 2);
    fg[0] += weights[1] * CppAD::pow(vars[epsi_start + t], 2);
    fg[0] += weights[2] * CppAD::pow(vars[v_start + t] - ref_v, 2);
  }

  // Minimize the use of actuators
  for (int t = 0; t < N - 1; ++t)
  {
    fg[0] += weights[3] * CppAD::pow(vars[delta_start + t], 2);
    fg[0] += weights[4] * CppAD::pow(vars[a_start + t], 2);
  }

  // Minimize the value gap between sequential actuators
  for (int t = 0; t < N - 2; ++t)
  {
    fg[0] += weights[5] * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += weights[6] * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
  }

  //
  // Setup Model Constraints
  //

  // Initial constraints
  //
  // We add 1 to each of the starting indices due to cost being located at
  // index 0 of `fg`.
  // This bumps up the position of all the other values.
  fg[1 + x_start] = vars[x_start];
  fg[1 + y_start] = vars[y_start];
  fg[1 + psi_start] = vars[psi_start];
  fg[1 + v_start] = vars[v_start];
  fg[1 + cte_start] = vars[cte_start];
  fg[1 + epsi_start] = vars[epsi_start];

  // The rest of the constraints
  for (int t = 1; t < N; ++t)
  {

    // The state at time t+1
    CppAD::AD<double> x1 = vars[x_start + t];
    CppAD::AD<double> y1 = vars[y_start + t];
    CppAD::AD<double> psi1 = vars[psi_start + t];
    CppAD::AD<double> v1 = vars[v_start + t];
    CppAD::AD<double> cte1 = vars[cte_start + t];
    CppAD::AD<double> epsi1 = vars[epsi_start + t];

    // The state at time t
    CppAD::AD<double> x0 = vars[x_start + t - 1];
    CppAD::AD<double> y0 = vars[y_start + t - 1];
    CppAD::AD<double> psi0 = vars[psi_start + t - 1];
    CppAD::AD<double> v0 = vars[v_start + t - 1];
    CppAD::AD<double> cte0 = vars[cte_start + t - 1];
    CppAD::AD<double> epsi0 = vars[epsi_start + t - 1];

    // Only consider the actuation at time t
    CppAD::AD<double> delta0 = vars[delta_start + t - 1];
    CppAD::AD<double> a0 = vars[a_start + t - 1];

    CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0;
    CppAD::AD<double> psides0 = CppAD::atan(coeffs[1]);

    // The idea here is to constraint this value to be 0.

    /**Setup the rest of the model constraints
     * The equations for the model:
     * x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
     * y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
     * psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
     * v_[t+1] = v[t] + a[t] * dt
     * cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
     * epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
     */

    fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
    fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
    fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
  }
}

} // namespace mpc
} // namespace agv