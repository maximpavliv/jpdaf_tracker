/*
 *  JPDAFTracker 
 *  Copyright 2017 Andrea Pennisi
 *
 *  This file is part of JPDAFTracker and it is distributed under the terms of the
 *  GNU Lesser General Public License (Lesser GPL)
 *
 *
 *
 *  JPDAFTracker is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  JPDAFTracker is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with JPDAFTracker.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  JPDAFTracker has been written by Andrea Pennisi
 *
 *  Please, report suggestions/comments/bugs to
 *  andrea.pennisi@gmail.com
 *
 */

#ifndef _TRACK_H_
#define _TRACK_H_

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <jpdaf_tracker/tracker_param.h>
#include <jpdaf_tracker/kalman.h>


namespace jpdaf
{
  class Track
  {
    public:
      Track(const float& x, const float& y, const float& vx, const float& vy, TrackerParam params);
      void predict(float dt);
      void setId(const int& _id)
      {
	    id = _id;
      }
      const int getId() const
      {
	    return id;
      }
      const Eigen::Matrix2f S() const
      {
    	return KF->getS();
      }
      void gainUpdate();
      void update(const std::vector<Detection> detections, std::vector<float> beta, float beta_0);
      bool isDeprecated()
      {
    	return noDetections >= maxMissedRate;
      }
    private:
      int id;
      int maxMissedRate;
      int minAcceptanceRate;
      std::shared_ptr<Kalman> KF;
      int noDetections;
      int life_time;
  };
}

#endif
