#ifndef JPDAF_NODE_HPP_
#define JPDAF_NODE_HPP_

#include <ros/ros.h>

#include <math.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <image_transport/image_transport.h>

#include <jpdaf_tracker/tracker_param.h>
#include <jpdaf_tracker/detection.h>
#include <jpdaf_tracker/track.h>
#include <jpdaf_tracker/hungarian_alg.h>

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <jpdaf_tracker_msgs/Track.h>
#include <jpdaf_tracker_msgs/Tracks.h>

namespace jpdaf {

class Node {
    public:
      Node(const ros::NodeHandle nh, const ros::NodeHandle pnh);
      
    private:
        //ros node handle
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        image_transport::ImageTransport it_;


        ros::Subscriber detection_sub_;
        ros::Subscriber image_sub_;
        ros::Subscriber pose_sub_;
        //ros::Subscriber mocap_sub_;

        bool track_init;
        std::vector<darknet_ros_msgs::BoundingBoxes> bounding_boxes_msgs_buffer_;
        //std::vector<nav_msgs::Odometry> gt_odom_buffer_;
        std::vector<sensor_msgs::ImageConstPtr> image_buffer_;
        std::vector<geometry_msgs::PoseStamped> pose_buffer_;

        image_transport::Publisher image_pub_;
        ros::Publisher tracks_pub_;

        double last_timestamp;

        std::vector<Track> tracks_;
        std::vector<int> lost_tracks;

        std::vector<Detection> prev_unassoc_detections;

        TrackerParam params;

        ros::Timer update_timer;

//----------------------------


        void detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes);
        void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);

        void poseCallback(const geometry_msgs::PoseStamped& pose_msg);
        //void gtCallback(const nav_msgs::OdometryConstPtr& msg);
        
        void timer_callback(const ros::TimerEvent& event);
//        void timer_callback(const ros::TimerEvent& event);

        void track(bool called_from_detection);

        void compute_timescaled_orientation_shift_flush_pose(void);

        void publishTracks();

        cv::Mat_<int> association_matrix(const std::vector<Detection> detections);

        void manage_new_old_tracks(std::vector<Detection> detections, std::vector<double> betas_0, std::vector<double> alphas_0);
        std::vector<Track> create_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections);

        std::vector<Detection> get_detections(const darknet_ros_msgs::BoundingBoxes last_detection);

        std::vector<double> compute_beta(int track_nb, std::vector<cv::Mat_<int>> hypothesis_matrices, std::vector<double> hypothesis_probabilities);

        std::vector<double> compute_alphas_0(std::vector<cv::Mat_<int>> hypothesis_mats, std::vector<double> hypothesis_probs);

        std::vector<double> compute_betas_0(std::vector<cv::Mat_<int>> hypothesis_mats, std::vector<double> hypothesis_probs);

        std::vector<cv::Mat_<int>> generate_hypothesis_matrices(cv::Mat_<int> assoc_mat);

        std::vector<double> compute_probabilities_of_hypothesis_matrices(std::vector<cv::Mat_<int>> hypothesis_matrices, std::vector<Detection> detections);

        double probability_of_hypothesis_unnormalized(cv::Mat_<int> hypothesis, std::vector<Detection> detections);

        cv::Mat_<int> tau(cv::Mat_<int> hypothesis);//THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
        cv::Mat_<int> delta(cv::Mat_<int> hypothesis);    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED

        void draw_tracks_publish_image(const sensor_msgs::ImageConstPtr last_image);


        std::vector<int> get_nonzero_indexes_row(cv::Mat_<int> mat);

};

} 

#endif 
