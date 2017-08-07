#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "polyutil.hpp"

using CppAD::AD;

// TODO: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.1;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t psi_error_start = cte_start + N;
const size_t delta_start = psi_error_start + N;
const size_t acc_start = delta_start + N - 1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double wheel_base = 2.67;

const double ref_vel = 100.0;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // state is [x, y, psi, v, cross track error, psi error]
    // control value are [delta, acceleration]

    // set up the cost
    fg[0] = 0.0;


    // cost based on reference state
    for(size_t t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 40 * CppAD::pow(vars[psi_error_start + t], 2);
      fg[0] += 0.1 * CppAD::pow(vars[v_start + t] - ref_vel, 2);
    }

    // cost for actuators
    // i.e. prefer small steering angles and acceleration
    for(size_t t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[acc_start + t], 2);
    }

    // minimize difference between sequential control changes
    // i.e. steer and accelerate smoothly
    for(size_t t = 0; t < N - 2; t++) {
      fg[0] += 10 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[acc_start + t + 1] - vars[acc_start + t], 2);
    }

    // minimize centripetal acceleration: v**2 * kappa
    // i.e. slower  curves
    // TODO(see--): Allow arbitary polynomial order
    assert(coeffs.size() == 4);
    for(size_t t = 0; t < N; t++) {
      AD<double> x = vars[x_start + t];

      AD<double> dy =
          coeffs[1]
          + 2 * coeffs[2] * x
          + 3 * coeffs[3] * x * x;
      AD<double> ddy =
              2 * coeffs[2]
              + 6 * coeffs[3] * x;
      AD<double> kappa = ddy / CppAD::pow(1.0 + dy * dy, 1.5);
      AD<double> v = vars[v_start + t];
      fg[0] += 0.03 * CppAD::pow(v * v * kappa, 2);
    }

    // remaining constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + psi_error_start] = vars[psi_error_start];

    for(size_t t = 1; t < N; t++) {
      // unpack state at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      // We compute cte0 again using our fitted polynom
      // AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> psi_error0 = vars[psi_error_start + t - 1];

      // unpack control vector at time t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> acc0 = vars[acc_start + t - 1];
      // unpack state at time t + 1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> psi_error1 = vars[psi_error_start + t];

      // evaluate fitted polynom
      AD<double> f0 =
          coeffs[0]
          + coeffs[1] * x0
          + coeffs[2] * x0 * x0
          + coeffs[3] * x0 * x0 * x0;
      AD<double> psi_des0 = CppAD::atan(
            coeffs[1]
            + 2 * coeffs[2] * x0
            + 3 * coeffs[3] * x0 * x0);

      // use our kinematic motion model
      // x
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      // y
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      // psi
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 / wheel_base * delta0 * dt);
      // velocity
      fg[1 + v_start + t] = v1 - (v0 + acc0 * dt);
      // cross track error
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(psi_error0) * dt));
      // error psi
      fg[1 + psi_error_start + t] =
          psi_error1 - ((psi0 - psi_des0) - v0 / wheel_base * CppAD::tan(delta0) * dt);

    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N - 1);
  // TODO: Set the number of constraints
  //
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[psi_error_start] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for(size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for(size_t i = delta_start; i < acc_start; i++) {
    vars_lowerbound[i] = -0.3;
    vars_upperbound[i] = 0.3;
  }
  for(size_t i = acc_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = state[0];
  constraints_upperbound[x_start] = state[0];

  constraints_lowerbound[y_start] = state[1];
  constraints_upperbound[y_start] = state[1];

  constraints_lowerbound[psi_start] = state[2];
  constraints_upperbound[psi_start] = state[2];

  constraints_lowerbound[v_start] = state[3];
  constraints_upperbound[v_start] = state[3];

  constraints_lowerbound[cte_start] = state[4];
  constraints_upperbound[cte_start] = state[4];

  constraints_lowerbound[psi_error_start] = state[5];
  constraints_upperbound[psi_error_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  if(!ok) {
    std::cout << "IPOPT: Something went wrong!" << std::endl;
    exit(0);
  }

  // set x and y values
  x_vals_.resize(N);
  y_vals_.resize(N);
  for(size_t i = 0; i < N; i++) {
    x_vals_[i] = solution.x[x_start + i];
    y_vals_[i] = solution.x[y_start + i];
  }


  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << " psi_error: " << solution.x[psi_error_start] << " cte: " << solution.x[cte_start]<< std::endl;
  // account for latency
  double latency_corrected_steer =
      (solution.x[delta_start] + solution.x[delta_start +1]) / 2.0;
  double latency_corrected_acc =
      (solution.x[acc_start] + solution.x[acc_start +1]) / 2.0;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  // return second control vector to account for latency
  return {latency_corrected_steer, latency_corrected_acc};
}
