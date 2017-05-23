#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
  
private:
  // number of sigma points
  int n_sigma_;
  
  // Measurement covariance matrixes
  MatrixXd R_radar_;
  MatrixXd R_lidar_;
  
  // Xsig_pred - mean(Xsig_pred) --> used in predict and update, added as a member for caching result
  MatrixXd Xsig_minus_x_;
  
  // Members used in Update step
  MatrixXd Zsig_pred_;
  
  /**
   * Normalize angle between -Pi and Pi
   * @param angle The angle to be normalized
   * @return the normalized angle
   */
  double NormalizeAngle(double angle);
  
  /**
   * Normalize angles values from a vector "close" to the first value_comp
   * @param angle_vector the list of angle to normalized
   * @return a vector of angle arround the first value_comp
   *
   * Example:
   *   angle_vector = [PI-0.1, PI -PI+0.1] -> [PI-0.1, PI, PI+0.1]
   */
  VectorXd NormalizeAngleArround(const VectorXd &angle_vector);
  
  /**
   * Common part of the Update step
   * @param Zsig_minus_z predicted sigma measurement minus mean of predicted measurement
   * @return NIS value
   */
   double UpdateCommon(const VectorXd &z, const MeasurementPackage::SensorType type);
  
};

#endif /* UKF_H */
