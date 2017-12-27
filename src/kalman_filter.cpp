#include "tools.h"
#include "kalman_filter.h"
#include "spdlog/spdlog.h"
#include <iostream>

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
  auto log = spdlog::stdout_color_mt("ekf_log");
  log->info("Initializing variables");
  Tools calc;
  VectorXd hx = calc.CartesianToPolar(x_);
  log->info("hx converted");
  std::cout << hx[0] << hx.size();
  for (int i = 0; i < hx.size(); ++i){
    log->info("hx({}) {}", i, hx[i]);
  }
  for (int i = 0; i < z.size(); ++i){
    log->info("z({}) {}", i, z[i]);
  }
  VectorXd y = z - hx;
  log->info("error calculated");
  y(1) = calc.NormalizePhi(y(1));
  log->info("error normalized");
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  log->info("S & K calculated");
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
  log->info("x & P updated");
}
