#include<jpdaf_tracker/node.h>

using namespace std;

namespace jpdaf {


Node::Node(ros::NodeHandle nh, ros::NodeHandle nh_priv):
    nh_(nh),
    nh_priv_(nh_priv)
{
    TrackerParam params(nh_priv_);

    K_ << intrinsics_[0], 0., intrinsics_[2],
                                   0., intrinsics_[1], intrinsics_[3],
                                   0., 0., 1.f;


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
//        ROS_INFO("tracking called with time step %f, detection boxes nb: %d", time_step, number_detections);

        auto norm_detections = get_detections(last_detection);
    }

    last_timestamp = last_detection.header.stamp.toSec();
    bounding_boxes_msgs_buffer_.clear();
    image_buffer_.clear();
}

std::vector<Detection> Node::get_detections(const darknet_ros_msgs::BoundingBoxes last_detection)
{
    std::vector<Detection> norm_det;
    for(int i=0; i<(int)last_detection.bounding_boxes.size(); i++)\
    {
//        std::vector<cv::Point2f> det(2);
//        det[0] = cv::Point2f((float)last_detection.bounding_boxes[i].xmin, (float)last_detection.bounding_boxes[i].ymin);
//        det[1] = cv::Point2f((float)last_detection.bounding_boxes[i].xmax, (float)last_detection.bounding_boxes[i].ymax);
//        std::vector<cv::Point2f> undist_det(2);
//        cv::fisheye::undistortPoints(det, undist_det, K_, dist_coeff_);
//        ROS_INFO("detection: %f %f %f %f", undist_det[0].x, undist_det[0].y, undist_det[1].x, undist_det[1].y);
        Detection one_det(float(last_detection.bounding_boxes[i].xmin+last_detection.bounding_boxes[i].xmax)/2, 
                          float(last_detection.bounding_boxes[i].ymin+last_detection.bounding_boxes[i].ymax)/2, 
                          last_detection.bounding_boxes[i].xmax-last_detection.bounding_boxes[i].xmin, 
                          last_detection.bounding_boxes[i].ymax-last_detection.bounding_boxes[i].ymin);
        norm_det.push_back(one_det);
    }
    return norm_det;
}


}
