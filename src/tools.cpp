#ifndef M_PI
  #define M_PI 3.1415926535897;
#endif
#include <cmath>
#include <iostream>
#include "tools.h"
#include "spdlog/spdlog.h"

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

VectorXd Tools::CartesianToPolar(const VectorXd& x_state) {
  VectorXd hx(3);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho = std::pow(std::pow(px,2) + std::pow(py,2), 0.5);
  float rho_dot;

  if (rho < 0.032) {
    hx.setZero();
  } else {
    hx << rho,
          NormalizePhi(std::atan2(py,px)),
          (px*vx + py*vy)/rho;
  }
  return hx;
}

float Tools::NormalizePhi(float phi) {
  return phi - 2*M_PI*std::floor((phi + M_PI) / (2*M_PI));
}

void Tools::PrintMatrix(std::string name, const MatrixXd &m) {
  auto console = spdlog::get("console");
  console->info("Printing  {}x{} matrix {}", m.rows(), m.cols(), name);
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) {
      console->info("\t{}\t{},{}\t{}", name, i, j, m(i, j));
    }
  }
  console->info("----------------------------");
}

void Tools::PrintVector(std::string name, const VectorXd &v) {
  auto console = spdlog::get("console");
  console->info("Printing vector {}", name);
  for (int i = 0; i < v.size(); ++i) {
    console->info("\t{}\t{}\t{}", name, i, v(i));
  }
  console->info("----------------------------");
}
