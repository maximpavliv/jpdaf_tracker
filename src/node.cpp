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
    track_init = true;
    
    for(int i=1; i<=params.nb_drones; i++)
    {
        lost_tracks.push_back(i);
    }

    ROS_INFO("Node initialized successfully");
}



void Node::detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes)
{
    bounding_boxes_msgs_buffer_.push_back(*bounding_boxes);
    if(image_buffer_.size() != 0)
        track();
}
void Node::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    image_buffer_.push_back(*img_msg);
    if(bounding_boxes_msgs_buffer_.size() != 0)
        track();
}
/*void Node::gtCallback(const nav_msgs::OdometryConstPtr& msg)
{
    gt_odom_buffer_.push_back(*msg);
    ROS_INFO("Received new mocap msg");
    gt_odom_buffer_.clear();
}*/
void Node::track()
{
    auto last_image = image_buffer_.back();
    auto last_detection = bounding_boxes_msgs_buffer_.back();
    int number_detections = last_detection.bounding_boxes.size();   

    if(track_init)
    {
        track_init = false;
    }
    else
    {
        double time_step = last_detection.header.stamp.toSec() - last_timestamp;
        ROS_INFO("tracking called with time step %f, detection boxes nb: %d", time_step, number_detections);

        //detections = getDetections in transformed coordinates
    }

    last_timestamp = last_detection.header.stamp.toSec();
    bounding_boxes_msgs_buffer_.clear();
    image_buffer_.clear();
}


}
