#ifndef M_PI
  #define M_PI 3.1415926535897;
#endif
#include "catch.hpp"
#include "../tools.cpp"

TEST_CASE("RMSE of invalid length is 0", "[rmse]") {
  Tools calc;
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;
  Eigen::VectorXd e(4);
  Eigen::VectorXd g(4);
  Eigen::VectorXd z(4);
  z.setZero();

  SECTION("RMSE of empty vector is 0") {
    REQUIRE(calc.CalculateRMSE(estimations, ground_truth) == z);
  }
  SECTION("RMSE of vectors of different length is 0") {
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(g);
    g << 1.1, 1.1, 0.3, 0.2;
    REQUIRE(calc.CalculateRMSE(estimations, ground_truth) == z);
  }
}

TEST_CASE("RMSE of two vectors calculated", "[rmse]") {
  Tools calc;
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;
  Eigen::VectorXd e(4);
  Eigen::VectorXd g(4);
  Eigen::VectorXd z(4);
  z.setZero();
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, 
                               ", ", ", ", "", "", " << ", ";");
  SECTION("RMSE of equal vectors is zero") {
    e << 1, 1, 1, 1;
    estimations.push_back(e);
    g << 1, 1, 1, 1;
    ground_truth.push_back(g);
    REQUIRE(calc.CalculateRMSE(estimations, ground_truth) == z);
  }

  SECTION("RMSE calculated") {
    Eigen::VectorXd res(4);
    Approx target = Approx(0.1).epsilon(0.01);
    
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);

    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    res = calc.CalculateRMSE(estimations, ground_truth);
    for(int i = 0; i < res.size(); ++i) {
      REQUIRE(res[i] == target);
    }
  }
}

TEST_CASE("Jacobian with small rho less returns 0", "[jacobian]"){
  Tools calc;
  Eigen::VectorXd x_predicted(4);
  Eigen::MatrixXd z(3,4);
  z.setZero();

  x_predicted << 0.001, 0.001, 0.2, 0.4;
  REQUIRE(calc.CalculateJacobian(x_predicted) == z);
}

TEST_CASE("Jacobian calculated", "[jacobian]") {
  Tools calc;
  Eigen::VectorXd x_predicted(4);
  Eigen::MatrixXd Hj(3,4);
  x_predicted << 1, 2, 0.2, 0.4;

  Hj = calc.CalculateJacobian(x_predicted);
  REQUIRE(Hj(0,0) == Approx(0.447214).epsilon(0.000001));
  REQUIRE(Hj(0,1) == Approx(0.894427).epsilon(0.000001));
  REQUIRE(Hj(0,2) == Approx(0).epsilon(0.000001));
  REQUIRE(Hj(0,3) == Approx(0).epsilon(0.000001));
  REQUIRE(Hj(1,0) == Approx(-0.4).epsilon(0.000001));
  REQUIRE(Hj(1,1) == Approx(0.2).epsilon(0.000001));
  REQUIRE(Hj(1,2) == Approx(0).epsilon(0.000001));
  REQUIRE(Hj(1,3) == Approx(0).epsilon(0.000001));
  REQUIRE(Hj(2,0) == Approx(0).epsilon(0.000001));
  REQUIRE(Hj(2,1) == Approx(0).epsilon(0.000001));
  REQUIRE(Hj(2,2) == Approx(0.447214).epsilon(0.000001));
  REQUIRE(Hj(2,3) == Approx(0.894427).epsilon(0.000001));
}

TEST_CASE("Normalize phi calculated" "[normalize-phi]") {
  Tools calc;
  
  REQUIRE(calc.NormalizePhi(0.25*M_PI) == Approx(0.25*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-0.25*M_PI) == Approx(-0.25*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(0.75*M_PI) == Approx(0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-0.75*M_PI) == Approx(-0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(1.25*M_PI) == Approx(-0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-1.25*M_PI) == Approx(0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(1.75*M_PI) == Approx(-0.25*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-1.75*M_PI) == Approx(0.25*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(2.25*M_PI) == Approx(0.25*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-2.25*M_PI) == Approx(-0.25*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(2.75*M_PI) == Approx(0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-2.75*M_PI) == Approx(-0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(3.25*M_PI) == Approx(-0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-3.25*M_PI) == Approx(0.75*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(3.75*M_PI) == Approx(-0.25*M_PI).epsilon(0.000001));
  REQUIRE(calc.NormalizePhi(-3.75*M_PI) == Approx(0.25*M_PI).epsilon(0.000001));
}
