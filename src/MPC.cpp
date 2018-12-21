#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// timestep length is 0.1 s with 7 steps in total
// Prediction horizon T is therefore 0.7 s
// For higher ref_v is makes sense to extend the prediction horizon
const int N = 12;       // Extending the prediction horizon works very well -> even the S-curves are detected as such
const double dt = 0.15; // Longer time lengths reduces oszillation and let the vehicle drive smoother

// Reference velocity (as explained in the lessons) -> algorithm works only with velocities that aren't too large (<70)
double ref_v = 50;

// initial old_state is given here (because both classes needs them)
const int x_start = 0;
const int y_start = x_start + N;
const int psi_start = y_start + N;
const int v_start = psi_start + N;
const int cte_start = v_start + N;
const int epsi_start = cte_start + N;
const int delta_start = epsi_start + N;
const int a_start = delta_start + N - 1;

class FG_eval
{
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars)
  {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (old_state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Here, the following parts are going to be implemented:
    // The starter code is provided in the solution of the quizzes here:
    // https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ee21948d-7fad-4821-b61c-0d69bbfcc425

    // fg[0] is the total cost of all components
    // -> it is summed up below in this variable
    fg[0] = 0;

    //****************************//
    //       Cost function        //
    //****************************//
    // Weights (penalty parameters)
    const int cte_weight = 1650;
    const int epsi_weight = 1850;
    const int v_weight = 1;
    const int delta_weight = 7;
    const int a_weight = 12;
    const int delta_diff_weight = 20;
    const int a_diff_weight = 25;

    // The part of the cost based on the reference old_state.
    for (int t = 0; t < N; t++)
    {
      fg[0] += cte_weight * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++)
    {
      fg[0] += delta_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += a_weight * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++)
    {
      fg[0] += delta_diff_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += a_diff_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //****************************************//
    //     Initialization & constraints       //
    //****************************************//
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int t = 1; t < N; t++)
    {
      // The old_state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The old_state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // In the github solution of the quiz are the calculation just for one dimension given, here more dimensions are needed
      // Idea taken from https://github.com/darienmt/CarND-MPC-Project-P5
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd old_state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Deriving the values from the old_state
  double x = old_state[0];
  double y = old_state[1];
  double psi = old_state[2];
  double v = old_state[3];
  double cte = old_state[4];
  double epsi = old_state[5];

  // TODO: Set the number of model variables (includes both old_states and inputs).
  // For example: If the old_state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // n_vars= N * old_state vector size + (N-1) * 2 actuactors (steering & acceleration)
  const int n_vars = N * old_state.size() + (N - 1) * 2;
  const int n_constraints = N * old_state.size();

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial old_state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
    vars[i] = 0;

  std::cout << vars << std::endl;
  //*****************************************//
  // Setting lower and upper limits for vars //
  //*****************************************//

  // Hyperparameter also taken from the solution:
  // https://github.com/udacity/CarND-MPC-Quizzes
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  // Upper accleration modified -> not so much acceleration needed here
  for (int i = a_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 0.7;
  }

  //************************************************//
  // Setting lower and upper limits for constraints //
  //************************************************//

  // Should be 0 besides initial old_state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Setting the initial values to other than 0. Idea taken from solution repo:
  // https://github.com/udacity/CarND-MPC-Quizzes
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

  // Cost
  auto cost = solution.obj_value;
  std::cout << "\nCost: " << cost << std::endl;

  // Return first actuactor values
  vector<double> resultvector;

  // steering_angle and acceleration
  resultvector.push_back(solution.x[delta_start]);
  resultvector.push_back(solution.x[a_start]);

  for (int i = 0; i < N - 1; i++)
  {
    // x and y values
    resultvector.push_back(solution.x[x_start + i + 1]);
    resultvector.push_back(solution.x[y_start + i + 1]);
  }

  return resultvector;
}

// Calculate the old_state with the latency taken into account
// Calulations are based on kinematic equations (based on previous lessons and wikipedia https://en.wikipedia.org/wiki/Equations_of_motion#Kinematic_quantities)
// Prerequisite: Assuming no change in derivate(steering_angle and throttle) is 0 during delay time
std::vector<double> MPC::StateWithLatency(std::vector<double> old_state, double steering_angle, double throttle)
{
  // Get the old state (as parameter)
  double px = old_state[0];
  double py = old_state[1];
  double psi = old_state[2];
  double v = old_state[3];

  // Equations for kinematic modelling
  double latency_s = latency_ms / 1000;
  double px_pred = px + (latency_s * v * cos(psi));
  double py_pred = py + (latency_s * v * sin(psi));
  double psi_pred = psi - ((latency_s * v * steering_angle) / Lf);
  double v_pred = v + latency_s * throttle;

  return {px_pred, py_pred, psi_pred, v_pred};
}