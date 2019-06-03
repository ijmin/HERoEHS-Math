/*
 * heroehs_kalman_filter.h
 *
 *  Created on: May 18, 2019
 *      Author: ijm
 */

#ifndef HEROEHS_MATH_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_HEROEHS_KALMAN_FILTER_H_
#define HEROEHS_MATH_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_HEROEHS_KALMAN_FILTER_H_



#include <Eigen/Dense>

namespace heroehs_math
{

class KalmanFilter {

public:

  KalmanFilter(
      const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H,
      double dt=0
  );
  ~KalmanFilter();

  void init();
  void init(const Eigen::VectorXd& x0, double t0);


  void update(const Eigen::VectorXd& y);
  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd F);

  void set(
      const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H,
      const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  Eigen::VectorXd state();
  double time();

private:
  Eigen::MatrixXd F, B, H, Q, R, K, P;


  int m, n;

  double t0, t;

  double dt;

  bool initialized;

  Eigen::MatrixXd I;

  Eigen::VectorXd x_hat, x_hat_new, u;
};

}




#endif /* HEROEHS_MATH_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_HEROEHS_KALMAN_FILTER_H_ */
