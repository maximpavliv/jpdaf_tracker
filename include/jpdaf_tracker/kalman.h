#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <jpdaf_tracker/tracker_param.h>
#include <jpdaf_tracker/detection.h>

namespace jpdaf
{
  class Kalman
  {
    public:
      Kalman(const float& x, const float& y, const float& vx, const float& vy, TrackerParam params);
      void predict(const float dt); 
      void gainUpdate();
      void update(const std::vector< Detection> detections, const std::vector<double> beta, double beta_0); //Added by Max
      inline const Eigen::Matrix2f getS() const
      {
    	return S;
      }
      const Eigen::Vector4f getUpdate()
      {
    	return x_update;
      }
      Eigen::Vector2f get_z_predict(){return z_predict;}
    private:
      Eigen::MatrixXf C;
      Eigen::Matrix2f R; //Proces measurement Covariance matrix
      Eigen::Matrix2f T; //Proces measurement Covariance matrix
      Eigen::Matrix2f S;
      Eigen::MatrixXf K; //Gain
      Eigen::Matrix4f P_predict; //Covariance Matrix predicted error
      Eigen::Vector4f x_predict;
      Eigen::Matrix4f P_update; //Covariance Matrix
      Eigen::Vector4f x_update;
      Eigen::Vector2f z_predict;
  };
}

#endif
