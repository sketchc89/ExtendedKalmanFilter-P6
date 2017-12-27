#ifndef TOOLS_H_
#define TOOLS_H_
#include <string>
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
   * A helper method to convert cartesian coordinates to polar coordinates.
   */
  VectorXd CartesianToPolar(const VectorXd& x_state);

  /**
   * A helper method to constrain all angles between -pi and +pi radians
   */
  float NormalizePhi(float phi);
  /**
   * A helper method to log values in matrix
   */
  void PrintMatrix(std::string name, const MatrixXd& m);

  /**
   * A helper method to log values in vector
   */
  void PrintVector(std::string name, const VectorXd& v);
};

#endif /* TOOLS_H_ */
