/* MPC.h

  Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology

  Model Predictive Control Class Declaration
*/

#ifndef MPC_H_
#define MPC_H_

#include <cmath>
#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "common/Eigen/Core"
#include "common/Eigen/QR"

namespace agv
{
namespace mpc
{

class MPC
{
 public:
  // Constructors
  MPC();
  MPC(const std::vector<double> &params, const std::vector<double> &weights, const std::vector<double> &constraints);
  // Destructor
  virtual ~MPC();

  // Solve the model given an initial state.
  // Return the next state and actuations as a std::vector.
  std::vector<double> Solve(const Eigen::VectorXd &x0, const Eigen::VectorXd &coeffs, double ref_v);

  class FG_eval
  {
  public:
    Eigen::VectorXd coeffs;      // Coefficients of the fitted polynomial.
    std::vector<double> weights; // Cost weights

    double dt;
    double Lf;
    double ref_v;

    size_t N;
    size_t M;
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_start;
    size_t cte_start;
    size_t epsi_start;
    size_t delta_start;
    size_t a_start;

    // Constructor
    FG_eval(Eigen::VectorXd coeffs, std::vector<double> weights, std::vector<size_t> sizes, double dt, double Lf, double ref_v);

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    void operator()(ADvector &fg, const ADvector &vars);

  }; // end of class FG_eval

 private:
  double dt;
  double L;
  double Lf;
  // double ref_v;
  double max_steering_angle;
  double max_deceleration;
  double max_acceleration;

  std::vector<double> weights;

  size_t N;
  size_t M;
  size_t x_start;
  size_t y_start;
  size_t psi_start;
  size_t v_start;
  size_t cte_start;
  size_t epsi_start;
  size_t delta_start;
  size_t a_start;
};

} // namespace mpc
} // namespace agv

#endif // MPC_H_
