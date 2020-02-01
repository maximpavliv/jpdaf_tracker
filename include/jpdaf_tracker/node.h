#ifndef JPDAF_NODE_HPP_
#define JPDAF_NODE_HPP_

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>

#include <jpdaf_tracker/tracker_param.h>
#include <jpdaf_tracker/detection.h>
#include <jpdaf_tracker/track.h>
#include <jpdaf_tracker/hungarian_alg.h>

//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/MagneticField.h>
//#include <std_srvs/Empty.h>
//#include <diagnostic_updater/diagnostic_updater.h>
//#include <diagnostic_updater/publisher.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/sync_policies/exact_time.h>

//#include <kr_attitude_eskf/AttitudeESKF.hpp>
//#include <kr_attitude_eskf/AttitudeMagCalib.hpp>

namespace jpdaf {

class Node {
    public:
      Node(const ros::NodeHandle nh, const ros::NodeHandle pnh);
      
    private:
        //ros node handle
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        ros::Subscriber detection_sub_;
        ros::Subscriber image_sub_;
        //ros::Subscriber mocap_sub_;

        bool track_init;
        std::vector<darknet_ros_msgs::BoundingBoxes> bounding_boxes_msgs_buffer_;
        //std::vector<nav_msgs::Odometry> gt_odom_buffer_;
        std::vector<sensor_msgs::Image> image_buffer_;

        image_transport::Publisher image_pub_;
        ros::Publisher tracks_pub_;

        double last_timestamp;

        std::vector<Track> tracks_;
        std::vector<int> lost_tracks;

        std::vector<Detection> prev_unassoc_detections;

        TrackerParam params;

//----------------------------


        void detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes);
        void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);
        //void gtCallback(const nav_msgs::OdometryConstPtr& msg);

        void track();


        cv::Mat_<int> association_matrix(const std::vector<Detection> detections);

        std::vector<int> not_associated_detections(cv::Mat_<int> q);

        void manage_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections);

        void manage_old_tracks();
        void validate_new_tracks();

        std::vector<Detection> get_detections(const darknet_ros_msgs::BoundingBoxes last_detection);
        

};

} 

#endif 
