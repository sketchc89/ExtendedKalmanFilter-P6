#ifndef M_PI
 #define M_PI 3.1415926535897;
#endif
#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <cmath>
#include <string>
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
    //console->set_level(spdlog::level::debug);
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
    MatrixXd F_in = MatrixXd::Identity(4,4);
    ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      console->debug("Initializing with RADAR");
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
      console->debug("Initializing with LIDAR");
      ekf_.x_ << measurement_pack.raw_measurements_(0), 
                 measurement_pack.raw_measurements_(1),
                 0,
                 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    console->debug("Initialization complete {}", ekf_.x_(0,0));
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  auto console = spdlog::get("console");
  console->info("Time: {}", measurement_pack.timestamp_);
  console->debug("Prediction step start");
  float dt;
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_ << std::pow(dt,4)*noise_ax/4, 0, std::pow(dt,3)*noise_ax/2, 0,
             0, std::pow(dt,4)*noise_ay/4, 0, std::pow(dt,3)*noise_ay/2,
             std::pow(dt,3)*noise_ax/2, 0, std::pow(dt,2)*noise_ax, 0,
             0, std::pow(dt,3)*noise_ay/2, 0, std::pow(dt,2)*noise_ay;

  std::string F_name="F", Q_name="Q"; 
  calc.PrintMatrix(F_name, ekf_.F_);
  calc.PrintMatrix(Q_name, ekf_.Q_);
  ekf_.Predict();
  console->debug("Prediction step end");

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  console->debug("Update start");
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    console->info("Updating with RADAR start");
    ekf_.H_ = calc.CalculateJacobian(ekf_.x_);

    std::string H_name="H", R_name="R";
    calc.PrintMatrix(H_name, ekf_.H_);
    ekf_.R_ = R_radar_;
    calc.PrintMatrix(R_name, ekf_.R_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    console->debug("Updating with RADAR end");
  } else {
    console->debug("Updating with LIDAR start");
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    console->debug("Updating with LIDAR end");
  }
  console->debug("Update end");
  
  std::string x_name="x", P_name="P";
  calc.PrintMatrix(x_name, ekf_.x_);
  calc.PrintMatrix(P_name, ekf_.P_);
}
