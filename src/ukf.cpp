#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  // dimension of state
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // Value setted according to the discussion in the lessons (max expected: 6m/s^2 / 2)
  // And verified by monitoring the NIS
  std_a_ = 1.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // Value setted by monitoring the NIS value, and thinking about the rotation rate of vehicle
  std_yawdd_ = M_PI/8;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  Done:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  // To force the state initialization with the first data
  is_initialized_ = false;
  
  // state augmentation: Adding 2: linear acceleration and yaw acceleration
  n_aug_ = n_x_ + 2;
  
  // spreading parameter
  lambda_ = 3 - n_aug_;
  
  // number of sigma points
  n_sigma_ = 2*n_aug_ + 1;
  
  // weight initialisation
  weights_ = VectorXd(n_sigma_);
  weights_.fill(1. / (2.*(lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  
  // create predicted sigma matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sigma_);
  
  // R matrixes, based on values defined above
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_, 0,           0,
              0,         std_radphi_, 0,
              0,         0,           std_radrd_;
  //R_radar_ = R_radar_*R_radar_; // works because diagonal
  
  R_lidar_ = MatrixXd(2,2);
  R_lidar_ << std_laspx_, 0,
              0,          std_laspy_;
  R_lidar_ = R_lidar_*R_lidar_; // works because diagonal
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  Done:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  // Check if it's a sensor that we use
  if (!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) return;
  if (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) return;
  
  // Initialise the state using the first measurment if it's not initialized yet 
  if (!is_initialized_) {
    // initial timestamp
    time_us_ = meas_package.timestamp_;
    
    double px=0., py=0., v=0., yaw=0., yaw_d=0.;
    switch (meas_package.sensor_type_) {
      case MeasurementPackage::LASER:
        // LIDAR gives directly px, py
        px = meas_package.raw_measurements_[0];
        py = meas_package.raw_measurements_[1];
        break;
      case MeasurementPackage::RADAR:
        // RADAR gives range, bearing and radial speed
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        double rho_d = meas_package.raw_measurements_[2];
        
        px = rho * cos(phi);
        py = rho * sin(phi);
        // design choice: I use the measured speed to partially initialize v and psi,
        // using the measured speed for v, and the direction of the measured speed for yaw
        v = fabs(rho_d);
        yaw = rho_d > 0 ? phi : -phi;
        break;
    }
    // initial state
    x_ << px, py, v, yaw, yaw_d;
    
    // initial state covariance matrix, ok confidence on px,py, large uncertainty on speed, yaw, yaw speed
    P_ << 0.0225, 0,      0, 0, 0,
          0,      0.0225, 0, 0, 0,
          0,      0,      5, 0, 0,
          0,      0,      0, 1, 0,
          0,      0,      0, 0, 0.5;
    
    is_initialized_ = true;
    
    // no prediction or update to be done
    return;
  }
  
  // Prediction step
  double dt = (meas_package.timestamp_ - time_us_) / 1.e6;
  
  // skip prediction if dt is more than 1 ms (example: if we had a case where we 
  // get lidar and radar data simultaneously, only one prediction step
  if (dt > 1e-3) {
    Prediction(dt);
    time_us_ = meas_package.timestamp_;
  }
    
  // Update step, with the corresponding sensor
  switch (meas_package.sensor_type_) {
  case MeasurementPackage::LASER:
    UpdateLidar(meas_package);
    break;
  case MeasurementPackage::RADAR:
    UpdateRadar(meas_package);
    break;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Done:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // state augmentation: add linear and rotational acceleration
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  // Mean of acceleration and rot. acceleration = 0
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;
  
  // Covariance matrix augmentation
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_*std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;
  
  // P_aug square root
  MatrixXd A = P_aug.llt().matrixL();
  
  // sigma point generation
  MatrixXd Xsig = MatrixXd(n_aug_, n_sigma_);
  Xsig.col(0) = x_aug;
  for (int i=0; i<n_aug_; ++i) {
    VectorXd delta_x = sqrt(lambda_+n_aug_) * A.col(i);
    Xsig.col(1+i) =        x_aug + delta_x;
    Xsig.col(1+n_aug_+i) = x_aug - delta_x;
  }
  
  // sigma point prediction according to CTRV model
  for (int i=0; i<n_sigma_; ++i) {
    // for convenience, get values with userfriendly variable
    double px     = Xsig(0,i);
    double py     = Xsig(1,i);
    double v      = Xsig(2,i);
    double yaw    = Xsig(3,i);
    double yaw_d  = Xsig(4,i);
    double a      = Xsig(5,i);
    double yaw_dd = Xsig(6,i);
    
    // two case, with or without yaw speed
    if (fabs(yaw_d) < 1e-6) {
      Xsig_pred_(0,i) = px + v * cos(yaw) * delta_t;
      Xsig_pred_(1,i) = py + v * sin(yaw) * delta_t;
    } else {
      Xsig_pred_(0,i) = px + v/yaw_d * (sin(yaw + yaw_d*delta_t) - sin(yaw));
      Xsig_pred_(1,i) = py + v/yaw_d * (cos(yaw) - cos(yaw + yaw_d*delta_t));
    }
    // add noise
    double delta_t_2 = delta_t*delta_t;
    Xsig_pred_(0,i) += 0.5 * a * delta_t_2 * cos(yaw);
    Xsig_pred_(1,i) += 0.5 * a * delta_t_2 * sin(yaw);
    
    // speed, yaw, yaw_d, including noise
    Xsig_pred_(2,i) = v + a * delta_t;
    Xsig_pred_(3,i) = yaw + yaw_d * delta_t + 0.5 * yaw_dd * delta_t_2;
    Xsig_pred_(4,i) = yaw_d + delta_t * yaw_dd;
  }
  
  // mean state computation ( = predicted state)
  // I use the function NormalizeAngleArround here, to handle case where the angles would be
  // for example -PI+eps and PI-eps, giving a mean of 0, where it should be pi
  Xsig_pred_.row(3) = NormalizeAngleArround(Xsig_pred_.row(3));
  x_ = Xsig_pred_ * weights_;
  x_(3) = NormalizeAngle(x_(3));
  
  // predicted covariance computation
  Xsig_minus_x_ =  Xsig_pred_.colwise() - x_;
  Xsig_minus_x_.row(3) = NormalizeAngleArround(Xsig_minus_x_.row(3));
  P_.fill(0.);
  for (int i=0; i<n_sigma_; ++i) {
    P_ += weights_(i) * Xsig_minus_x_.col(i) * Xsig_minus_x_.col(i).transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  DONE:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  // Predicted sigma points measurements
  // (ladar gives us px,py, no computation needed, only extracting values)
  Zsig_pred_ = MatrixXd(2, n_sigma_);
  for (int i=0; i<n_sigma_; ++i) {    
    Zsig_pred_.col(i) << Xsig_pred_(0,i), Xsig_pred_(1,i);;
  }
  
  // Common update step
  NIS_laser_ = UpdateCommon(meas_package.raw_measurements_, MeasurementPackage::LASER);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  DONE:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  // measurement prediction
  Zsig_pred_ = MatrixXd(3, n_sigma_);
  for (int i=0; i<n_sigma_; ++i) {
    double px, py, yaw, v;
    px  = Xsig_pred_(0,i);
    py  = Xsig_pred_(1,i);
    v   = Xsig_pred_(2,i);
    yaw = Xsig_pred_(3,i);
    
    if (fabs(px) < 1e-4 && fabs(py) < 1e-4) {
      px = px>0 ? 1e-4:-1e-4;
      py = py>0 ? 1e-4:-1e-4;
    }
    
    double rho, phi, rho_d;
    rho = sqrt(px*px + py*py);
    phi = atan2(py, px);
    rho_d = (px*cos(yaw)*v + py*sin(yaw)*v) / rho;
    
    Zsig_pred_.col(i) << rho, phi, rho_d;
  }
  Zsig_pred_.row(1) = NormalizeAngleArround(Zsig_pred_.row(1));
  
  // Common update step
  NIS_radar_ = UpdateCommon(meas_package.raw_measurements_, MeasurementPackage::RADAR);
  
}

// This function handle steps that are common in update function
// (everything after the Zsig_pred computation)
double UKF::UpdateCommon(const VectorXd &z, const MeasurementPackage::SensorType type) {
  // predicted mean, covariance
  VectorXd z_pred = Zsig_pred_ * weights_;
  
  // TODO refactor this (identical with predict)
  MatrixXd Zsig_minus_z_pred = Zsig_pred_.colwise() - z_pred;

  MatrixXd S = MatrixXd(z_pred.size(), z_pred.size());
  S.fill(0.);
  for (int i=0; i<n_sigma_; ++i) {
    S += weights_(i) * Zsig_minus_z_pred.col(i) * Zsig_minus_z_pred.col(i).transpose();
  }
  if (type == MeasurementPackage::LASER) {
    S += R_lidar_;
  } else {
    S += R_radar_;
  }
  
  // Cross corelation matrix
  MatrixXd T = MatrixXd(n_x_, z_pred.size());
  T.fill(0.);
  for (int i=0; i<n_sigma_; ++i) {
      T += weights_(i) * Xsig_minus_x_.col(i) * Zsig_minus_z_pred.col(i).transpose();
  }
  
  // Kalman gain
  MatrixXd S_inv = S.inverse();
  MatrixXd K = T * S_inv; 
  
  // update state and covariance matrix
  VectorXd Z_k1_minus_zk1k = (z - z_pred);
  
  // Normalize angle if radar
  if (type == MeasurementPackage::RADAR) {
    Z_k1_minus_zk1k(1) = NormalizeAngle(Z_k1_minus_zk1k(1));
  }
  
  x_ += K * Z_k1_minus_zk1k;
  x_(3) = NormalizeAngle(x_(3));
  
  P_ -= T * K.transpose();
  
  // nis computation
  return Z_k1_minus_zk1k.transpose() * S_inv * Z_k1_minus_zk1k;;
  
}

double UKF::NormalizeAngle(double angle) {
  double val = angle;
  while (val > M_PI) val -= 2*M_PI;
  while (val <-M_PI) val += 2*M_PI;
  return val;
}

/**
 * Normalize angles values from a vector "close" to the first value_comp
 * @param angle_vector the list of angle to normalized
 * @return a vector of angle arround the first value_comp
 *
 * Example:
 *   angle_vector = [PI-0.1, PI -PI+0.1] -> [PI-0.1, PI, PI+0.1]
 */
VectorXd UKF::NormalizeAngleArround(const VectorXd &angle_vector) {
  double first_value = NormalizeAngle(angle_vector(0));
  
  VectorXd result = VectorXd(angle_vector.size());
  result(0) = first_value;
  for (int i=1; i<angle_vector.size(); ++i) {
    double val = angle_vector(i);
      while (val > first_value+M_PI) val -= 2*M_PI;
      while (val < first_value-M_PI) val += 2*M_PI;
      result(i) = val;
  }
  return result;
}