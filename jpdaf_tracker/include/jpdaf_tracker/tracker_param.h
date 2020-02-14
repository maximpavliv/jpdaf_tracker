#ifndef _TRACKER_PARAM_H_
#define _TRACKER_PARAM_H_

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>


namespace jpdaf
{
  class TrackerParam
  {
    public:
      float pd;
      float gamma;
      double false_measurements_density;
      double beta_0_threshold;
      double alpha_0_threshold;
      int max_missed_rate;
      int min_acceptance_rate;
      Eigen::Matrix2f R;
      Eigen::Matrix2f T;
      Eigen::Matrix4f P_0;
      int nb_drones;
      float assoc_cost;

      float max_update_time_rate;

      TrackerParam(ros::NodeHandle nh_priv_);
      TrackerParam(){};

  };
}


#endif
