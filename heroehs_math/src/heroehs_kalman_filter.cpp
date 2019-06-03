/*
 * heroehs_kalman_filter.cpp
 *
 *  Created on: May 18, 2019
 *      Author: ijm
 */





#include <iostream>
#include <stdexcept>
#include <stdio.h>

#include "heroehs_math/heroehs_kalman_filter.h"


using namespace heroehs_math;

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    double dt
    )
: F(F), H(H),
  n(F.rows()), m(H.rows()), dt(dt), initialized(false),
  I(n, n), x_hat(n), x_hat_new(n), u(n)
{

  B=Eigen::MatrixXd::Zero(n,n);
  Q=Eigen::MatrixXd::Identity(n,n);
  R=Eigen::MatrixXd::Identity(m,m);
  P=Eigen::MatrixXd::Identity(n,n);

  I.setIdentity();
  u.setOnes();

  t=0;
  t0=0;
}

KalmanFilter::~KalmanFilter()
{

}

void KalmanFilter::init(const Eigen::VectorXd& x0,double t0)
{
  x_hat = x0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init()
{
  x_hat.setZero();
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y)
{
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = F * x_hat + B*u;
  P = F*P*F.transpose() + Q;
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat_new += K * (y - H*x_hat_new);
  P = (I - K*H)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd F)
{
  this->F = F;
  this->dt = dt;
  update(y);
}

void KalmanFilter::set(
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P
)
{
  this->F = F;
  this->H = H;
  this->B = B;
  this->Q = Q;
  this->R = R;
  this->P = P;
}

Eigen::VectorXd KalmanFilter::state()
{
  //printf("size : %d  %d\n",x_hat.rows(),x_hat.cols());
  return x_hat;
}

double KalmanFilter::time()
{
  return t;
}
