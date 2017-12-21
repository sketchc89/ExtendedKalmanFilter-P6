#include <cmath>
#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd> &estimations,
                              const std::vector<VectorXd> &ground_truth) {
  
  int est_sz = estimations.size();
  int grt_sz = ground_truth.size();

  VectorXd rmse(4);
  rmse.setZero();

  if (est_sz == 0 || est_sz != grt_sz) {
    return rmse;
  }
  

  for (int i = 0; i < est_sz; ++i) {
    VectorXd accum = estimations[i] - ground_truth[i];
    accum = accum.array()*accum.array();
    rmse += accum;
  }
  rmse /= est_sz;
  return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float rho = std::pow(std::pow(px,2) + std::pow(py,2), 0.5);
  if (rho < 0.032) {
    Hj.setZero();
    return Hj;
  }

  Hj << px/rho, py/rho, 0, 0,
        -py/std::pow(rho,2), px/std::pow(rho,2), 0, 0,
        py*(vx*py - vy*px)/std::pow(rho,3),
        px*(vy*px - vx*py)/std::pow(rho,3),
        px/rho, py/rho;
  return Hj;
}
