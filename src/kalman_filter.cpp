#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
    time_step++;
    
    cout << "Predicting ... " << endl;
    cout << "pre P_=" << P_ << endl;
    cout << "pre Q_=" << Q_ << endl;
    cout << "pre F_=" << F_ << endl;
    std::cout << "(k-1)x_'=" << endl << x_ << std::endl;
    x_ = F_ * x_;
    std::cout << "(k|k-1)x_'=" << endl << x_ << std::endl;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
//    cout << "!!!!!!!!!" << endl;
    cout << "Merging ... " << endl;
        std::cout << "z=" << endl << z << std::endl;
        std::cout << "P_=" << endl << P_ << std::endl;
        std::cout << "H_=" << endl << H_ << std::endl;
        std::cout << "R_=" << endl << R_ << std::endl;
    VectorXd y(2);
    MatrixXd S(2, 2);
    MatrixXd K(4, 2);
    MatrixXd I_(4, 4);
    
//    if (time_step <= thresh_step) {
//        x_(0) /= gain_factor;
//        x_(1) /= gain_factor;
//    }
    y << z - H_ * x_;
        std::cout << "y=" << endl << y << std::endl;
    S << H_ * P_ * H_.transpose() + R_;
        std::cout << "S=" << endl << S << std::endl;
    K << P_ * H_.transpose() * S.inverse();
        std::cout << "K=" << endl << K << std::endl;
    x_ = x_ + K * y;
        std::cout << "(k|k)x_=" << endl << x_ << std::endl;
    I_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    
        cout << "Merging ... " << endl;
    std::cout << "z=" << endl << z << std::endl;
    std::cout << "P_=" << endl << P_ << std::endl;
    std::cout << "H_=" << endl << H_ << std::endl;
    std::cout << "R_=" << endl << R_ << std::endl;
    VectorXd h_(3);
    VectorXd y(3);
    MatrixXd S(3, 3);
    MatrixXd K(4, 3);
    MatrixXd I_(4, 4);
    
    h_ << sqrt(x_(0) * x_(0) + x_(1) * x_(1)),
        atan2(x_(1), x_(0)),
        (x_(0) * x_(2) + x_(1) * x_(3)) / sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    std::cout << "h_=" << endl << h_ << std::endl;
    
//    if (time_step <= thresh_step) {
//        h_(0) /= gain_factor;
//    }
    y << z - h_;

    if (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
    }
    if (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
    }

    std::cout << "y=" << endl << y << std::endl;
    S << H_ * P_ * H_.transpose() + R_;
    std::cout << "S=" << endl << S << std::endl;
    K << P_ * H_.transpose() * S.inverse();
    std::cout << "K=" << endl << K << std::endl;
    x_ = x_ + K * y;
    std::cout << "(k|k)x_=" << endl << x_ << std::endl;
    I_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    P_ = (I_ - K * H_) * P_;
}
