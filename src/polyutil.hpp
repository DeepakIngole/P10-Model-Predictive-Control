#ifndef POLYUTIL_HPP
#define POLYUTIL_HPP

#include <cmath>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, double x);

// Evaluate angle
double angleeval(const Eigen::VectorXd& coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(
  const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);

#endif /* POLYUTIL_HPP */