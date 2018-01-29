#include "tools.h"
#include "kalman_filter.h"
#include "spdlog/spdlog.h"
#include <iostream>
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  auto log = spdlog::stdout_color_mt("ekf_log");
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_prev = H_ * x_;
  VectorXd y = z - z_prev;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  auto log = spdlog::get("ekf_log");
  //log->set_level(spdlog::level::debug);
  log->debug("Initializing variables");
  Tools calc;
  VectorXd hx = calc.CartesianToPolar(x_);
  log->debug("hx converted");
  if (hx.isZero(1e-3)) {
    log->warn("Object too close for RADAR to detect");
    return;
  }
  std::string hx_name = "hx";
  calc.PrintVector(hx_name, hx);
  std::string z_name = "z";
  calc.PrintVector(z_name, z);
  VectorXd y = z - hx;
  log->debug("error calculated");
  y(1) = calc.NormalizePhi(y(1));
  log->debug("error normalized");
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  log->debug("S & K calculated");
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
  log->debug("x & P updated");
}
