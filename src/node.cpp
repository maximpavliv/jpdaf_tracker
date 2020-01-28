#include<jpdaf_tracker/node.h>

using namespace std;

namespace jpdaf {


Node::Node(ros::NodeHandle nh, ros::NodeHandle nh_priv):
    nh_(nh),
    nh_priv_(nh_priv)
{
    TrackerParam params(nh_priv_);

    detection_sub_ = nh_priv_.subscribe("detection", 10, &Node::detectionCallback, this);
    image_sub_ = nh_priv_.subscribe("image", 10, &Node::imageCallback, this);
    //mocap_sub_ = nh_priv_.subscribe("gt", 10, &Node::gtCallback, this);


    ROS_INFO("Node initialized successfully");
}



void Node::detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes)
{
    bounding_boxes_msgs_buffer_.push_back(*bounding_boxes);
    ROS_INFO("Recieved new bounding boxes msg");
    bounding_boxes_msgs_buffer_.clear();
}
void Node::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    image_buffer_.push_back(*img_msg);
    ROS_INFO("Recieved new image msg");
    image_buffer_.clear();
}
/*
void Node::gtCallback(const nav_msgs::OdometryConstPtr& msg)
{
    gt_odom_buffer_.push_back(*msg);
    ROS_INFO("Received new mocap msg");
    gt_odom_buffer_.clear();
}
*/



}
