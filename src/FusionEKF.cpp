#ifndef M_PI
 #define M_PI 3.1415926535897;
#endif
#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <cmath>
#include <iostream>
#include "spdlog/spdlog.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  //console_ = spdlog::get("console");
  //console_->info("Welcome to spdlog");

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  Hj_.setZero();
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    auto console = spdlog::stdout_logger_mt("console");
    console->info("First measurement");
    console->info("Time: {}", measurement_pack.timestamp_);
    // first measurement
    cout << "EKF: " << endl;
    VectorXd x_in = VectorXd(4);
    x_in << 1, 1, 1, 1;
    MatrixXd P_in = MatrixXd(4,4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    MatrixXd Q_in = MatrixXd(4,4);
    Q_in.setZero();
    MatrixXd F_in = MatrixXd(4,4);
    F_in.setZero();
    ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      console->info("Initializing with RADAR");
      float rho = measurement_pack.raw_measurements_(0);
      float phi = calc.NormalizePhi(measurement_pack.raw_measurements_(1));
      float rho_dot = measurement_pack.raw_measurements_(2);
      ekf_.x_ << std::cos(phi) * rho,
                 std::sin(phi) * rho,
                 std::cos(phi) * rho_dot,
                 std::sin(phi) * rho_dot;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      console->info("Initializing with LIDAR");
      ekf_.x_ << measurement_pack.raw_measurements_(0), 
                 measurement_pack.raw_measurements_(1),
                 0,
                 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    console->info("Initialization complete {}", ekf_.x_(0,0));
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  auto console = spdlog::get("console");
  console->info("Time: {}", measurement_pack.timestamp_);
  console->info(measurement_pack.timestamp_);
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", ", ", "", "", " << ", ";");
  console->info("Prediction step start");
  float dt;
  float noise_ax = 9;
  float noise_ay = 9;
  dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;

  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_ << std::pow(dt,4)*noise_ax/4, 0, std::pow(dt,3)*noise_ax/2, 0,
             0, std::pow(dt,4)*noise_ay/4, 0, std::pow(dt,3)*noise_ay/2,
             std::pow(dt,3)*noise_ax/2, 0, std::pow(dt,2)*noise_ax, 0,
             0, std::pow(dt,3)*noise_ay/2, 0, std::pow(dt,2)*noise_ay;

  for (int i = 0; i < ekf_.F_.rows(); ++i){
    for (int j = 0; j < ekf_.F_.cols(); ++j){
      console->info("ekf_F_({}, {}) {}", i, j, ekf_.F_(i, j));
    }
  }
  for (int i = 0; i < ekf_.Q_.rows(); ++i){
    for (int j = 0; j < ekf_.Q_.cols(); ++j){
      console->info("ekf_Q_({}, {}) {}", i, j, ekf_.Q_(i, j));
    }
  }
  ekf_.Predict();
  console->info("Prediction step end");
  /*****************************************************************************
   *  Update
   ****************************************************************************/
  console->info("Update start");
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    console->info("Updating with RADAR start");
    ekf_.H_ = calc.CalculateJacobian(ekf_.x_);

    for (int i = 0; i < ekf_.H_.rows(); ++i){
      for (int j = 0; j < ekf_.H_.cols(); ++j){
        console->info("ekf_H_({}, {}) {}", i, j, ekf_.H_(i, j));
      }
    }
    ekf_.R_ = R_radar_;
    for (int i = 0; i < ekf_.R_.rows(); ++i){
      for (int j = 0; j < ekf_.R_.cols(); ++j){
        console->info("ekf_R_({}, {}) {}", i, j, ekf_.R_(i, j));
      }
    }
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    console->info("Updating with RADAR end");
  } else {
    console->info("Updating with LIDAR start");
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    console->info("Updating with LIDAR end");
  }
  console->info("Update end");

  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
