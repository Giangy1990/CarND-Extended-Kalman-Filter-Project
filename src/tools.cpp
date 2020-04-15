#include "tools.h"
#include <iostream>
#include <limits>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // input checks
  if (estimations.size() == 0){
      cout << "CalculateRMSE () - Error - estimation input vector has size == 0" << endl;
      return rmse;
  }
  if (estimations.size() != ground_truth.size()){
      cout << "CalculateRMSE () - Error - estimation and ground_truth have different size" << endl;
      return rmse;
  }
  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  // calculate the mean
  rmse = rmse/estimations.size();
  // calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // compute parameters
  float den1 = px*px + py*py;
  float den2 = sqrt(den1);
  float den3 = den1*den2;
  float num1 = py*vx - px*vy;
  float num2 = px*vy - py*vx;
  // check division by zero
  if (fabs(den1) < std::numeric_limits<float>::epsilon()){
    den1 = std::numeric_limits<float>::epsilon();
  }
  if (fabs(den2) < std::numeric_limits<float>::epsilon()){
    den2 = std::numeric_limits<float>::epsilon();
  }
  if (fabs(den3) < std::numeric_limits<float>::epsilon()){
    den3 = std::numeric_limits<float>::epsilon();
  }
  // compute jacobian
  Hj << px/den2, py/den2, 0, 0,
  		-py/den1, px/den1, 0, 0,
  		py*num1/den3, px*num2/den3, px/den2, py/den2;
  return Hj;
}
