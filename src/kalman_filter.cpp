#include "kalman_filter.h"

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
  MatrixXd S = H_ * P_ * H_.transpose() + R;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Tools calc;
  VectorXd hx = calc.CartesianToPolar(x_);
  MatrixXd Hj = calc.CalculateJacobian(z);
  VectorXd y = z - hx;
  y(1) = calc.NormalizePhi(y(1));
  MatrixXd S = Hj * P_ * Hj.transpose() + R;
  MatrixXd K = P_ * Hj.transpose() * S.inverse();
}
