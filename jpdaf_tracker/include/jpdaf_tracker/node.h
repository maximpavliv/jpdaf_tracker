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

//        double last_timestamp_ros_time_now_debug;

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

        void compute_timescaled_orientation_change_flush_pose(double detection_time_stamp);

        void publishTracks();

        Eigen::MatrixXf association_matrix(const std::vector<Detection> detections);

        void manage_new_old_tracks(std::vector<Detection> detections, std::vector<double> alphas_0, std::vector<double> betas_0);
        std::vector<Track> create_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections);

        std::vector<Detection> get_detections(const darknet_ros_msgs::BoundingBoxes last_detection);

        Eigen::MatrixXf compute_betas_matrix(std::vector<Eigen::MatrixXf> hypothesis_mats, std::vector<double> hypothesis_probs);

        //std::vector<double> compute_beta(int track_nb, std::vector<Eigen::MatrixXf> hypothesis_matrices, std::vector<double> hypothesis_probabilities);

        //std::vector<double> compute_alphas_0(std::vector<Eigen::MatrixXf> hypothesis_mats, std::vector<double> hypothesis_probs);
        //std::vector<double> compute_betas_0(std::vector<Eigen::MatrixXf> hypothesis_mats, std::vector<double> hypothesis_probs);

        std::vector<Eigen::MatrixXf> generate_hypothesis_matrices(Eigen::MatrixXf assoc_mat);

        std::vector<double> compute_probabilities_of_hypothesis_matrices(std::vector<Eigen::MatrixXf> hypothesis_matrices, std::vector<Detection> detections);

        double probability_of_hypothesis_unnormalized(Eigen::MatrixXf hypothesis, std::vector<Detection> detections);

        Eigen::MatrixXf tau(Eigen::MatrixXf hypothesis);//THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
        Eigen::MatrixXf delta(Eigen::MatrixXf hypothesis);    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED

        void draw_tracks_publish_image(const sensor_msgs::ImageConstPtr last_image);


        std::vector<int> get_nonzero_indexes_row(Eigen::MatrixXf mat);

};

} 

#endif 
