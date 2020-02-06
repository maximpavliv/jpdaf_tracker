#include <jpdaf_tracker/node.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "jpdaf_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  jpdaf::Node node(nh, pnh);


  ros::spin();
  return 0;

}
