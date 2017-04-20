#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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

  // NUMSENSORS is 2 for this case
  // The first components store H and R for KF used with LIDAR
  // The second components store Jacobian of H and R for EKF used with RADAR
  MatrixXd H_array[NUMSENSORS_];
  MatrixXd R_array[NUMSENSORS_];

  // initializing matrices
  H_array[LIDAR_] = MatrixXd(2, 4);
  H_array[RADAR_] = MatrixXd(3, 4);
  R_array[LIDAR_] = MatrixXd(2, 2);
  R_array[RADAR_] = MatrixXd(3, 3);

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  double std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_ = 0.03;
  //std_radphi_ = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ = 0.3;
  //std_radrd_ = 0.1;

  // measurement covariance matrices
  R_array[LIDAR_] << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
  R_array[RADAR_] << std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

  // measurement matrices, not used for UKF, uncomment for KF/EKF
	// H_array[LIDAR_] << 1, 0, 0, 0,
	// 		  0, 1, 0, 0;
  //
  // H_array[RADAR_] << 1, 0, 0, 0,
	// 		  0, 1, 0, 0,
  //       0, 0, 1, 0;

  // initialize state
  VectorXd x = VectorXd(5);
  x << 0, 0, 0, 0, 0;

  // initialize P, F, and Q and the filters
  MatrixXd P = MatrixXd(5, 5);
  P << 5, 0, 0, 0, 0,
        0, 5, 0, 0, 0,
        0, 0, 10, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 10;

  MatrixXd F = MatrixXd(4, 4);
  F << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  MatrixXd Q = MatrixXd(4, 4);

  // ekf_.Init(x, P, F,
  //     H_array, R_array, Q);

  // initialize the unscented Kalman Filter
  ukf_.Init(x, P, R_array);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

// processes each measurement depending if it's LIDAR or RADAR
// moved this to Fusion class so can easily transfer between using
// KF/EKF/UKF
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  cout << "Fusion: " << endl;
  if (!is_initialized_) {

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_prime = measurement_pack.raw_measurements_(2);

      // can choose between KF/EKF:
      // ekf_.x_ << rho*cos(phi), rho*sin(phi), rho_prime*cos(phi), rho_prime*sin(phi);

      // or UKF:
      // state vector is different for our ukf model
      // [px, py, v, phi, phidot], note: phidot not observed
      // the last two are placeholders and will get overwritten by augmentation
      if (fabs(rho) < 0.01)
      {
        return;
      }
      ukf_.x_ << rho*cos(phi), rho*sin(phi), rho_prime, phi, 0, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);

      // can choose between KF/EKF:
      // since velocities are unknown, they are initialized to zero,
      // but with large variances
      // ekf_.x_ << px, py, 0, 0;

      // or UKF:
      // state vector is different for our ukf model
      // [px, py, v, phi, phidot], note: v and phidot not observed
      // the last two are placeholders and will get overwritten by augmentation
      if (px*px+py*py < 0.01)
      {
        return;
      }
      float phi = atan2(py,px);
      ukf_.x_ << px, py, 0, phi, 0, 0, 0;
    }

    // initialize timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Update the state transition matrix F according to the new elapsed time in seconds

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // predict
  // choose between KF/EKF:
  // ekf_.Predict(dt);

  // or UKF:
  ukf_.Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    ukf_.UpdateRadar(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    // ekf_.UpdateKF(measurement_pack.raw_measurements_);
    ukf_.UpdateLidar(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ukf_.x_ << endl;
  cout << "P_ = " << ukf_.P_ << endl;
}
