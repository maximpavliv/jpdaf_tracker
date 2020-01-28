/*
 * kr_attitude_eskf.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of kr_attitude_eskf.
 *
 *  Created on: 17/6/2014
 *		  Author: gareth
 */

#include <jpdaf_tracker/node.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "jpdaf_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  jpdaf::Node node(nh, pnh);








  ros::spin();
  return 0;

}
