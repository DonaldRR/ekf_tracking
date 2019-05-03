#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    
    if (estimations.size() == 0) {
        return rmse;
    }
    
    if (ground_truth.size() == 0) {
        return rmse;
    }
    
    if (ground_truth.size() != estimations.size()) {
        return rmse;
    }

    for (int i=0; i< estimations.size(); i++) {
        VectorXd residual(4);
//        cout << "Estimation:" << estimations[i] <<endl;
        residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();

    for (int i=0; i<4; i++) {
        rmse(i) = sqrt(rmse(i));
    }

    cout << "RMSE:" << rmse << endl;

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */


    float c1 = sqrt(x_state(0) * x_state(0) + x_state(1) * x_state(1));
    float c2 = c1 * c1;
    float c3 = (x_state(2) * x_state(1) - x_state(3) * x_state(0)) / (c1 * c2);

    MatrixXd Hj(3, 4);

    if (fabs(c1) < 0.0001) {
        std::cout << "The Jacobian has denominator equal to zeros" << std::endl;
        return Hj;
    }
    else{
        Hj << x_state(0) / c1, x_state(1) / c1, 0, 0,
            -x_state(1) / c2, x_state(0) / c2, 0, 0,
            x_state(1) * c3 / (c1 * c2), - x_state(0) * c3 / (c1 * c2), x_state(0) / c1, x_state(1) / c1;
    }

    return Hj;
}
