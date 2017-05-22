#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  Done:
    * Calculate the RMSE here.
  */
  
  // Check input sizes
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    std::cout << "Tools::CalcalateRMSE - Error -- Input size 0 or doesn't match" << std::endl;
    VectorXd rmse(1);
    return rmse;
  }
  
  // Initialize to 0
  VectorXd rmse(estimations[0].size());
  rmse.fill(0.);
  
  // Accumulate the error
  std::vector<VectorXd>::const_iterator i1,i2;
  for (i1 = estimations.begin(), i2 = ground_truth.begin();
       i1 < estimations.end() && i2 < ground_truth.end();
       ++i1, ++i2) {
    VectorXd residual = *i1-*i2;
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // square root of average
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}
