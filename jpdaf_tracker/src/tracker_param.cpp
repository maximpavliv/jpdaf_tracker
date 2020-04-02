#include <jpdaf_tracker/tracker_param.h>
#include <iostream>

using namespace std;


namespace jpdaf
{
    TrackerParam::TrackerParam(ros::NodeHandle nh_priv_)
    {
        nh_priv_.getParam("pd", pd);
        nh_priv_.getParam("gamma", gamma);
        nh_priv_.getParam("false_measurements_density", false_measurements_density);
        nh_priv_.getParam("beta_0_threshold", beta_0_threshold);
        nh_priv_.getParam("alpha_0_threshold", alpha_0_threshold);
        nh_priv_.getParam("max_missed_rate", max_missed_rate);
        nh_priv_.getParam("min_acceptance_rate", min_acceptance_rate);
        nh_priv_.getParam("min_acceptance_rate", min_acceptance_rate);
        std::vector<float> R_vector;
        nh_priv_.getParam("R", R_vector);
        R << R_vector[0], 0, 0, R_vector[1];
        std::vector<float> T_vector;
        nh_priv_.getParam("T", T_vector);
        //T << T_vector[0], 0, 0, T_vector[1];
        T << T_vector[0], T_vector[1];
        std::vector<float> P_0_vector;
        nh_priv_.getParam("P_0", P_0_vector);
        P_0 << P_0_vector[0], 0, 0, 0,
               0, P_0_vector[1], 0, 0,
               0, 0, P_0_vector[2], 0,
               0, 0, 0, P_0_vector[3];
        nh_priv_.getParam("nb_drones", nb_drones);
        nh_priv_.getParam("assoc_cost", assoc_cost);

        nh_priv_.getParam("max_update_time_rate", max_update_time_rate);

        nh_priv_.getParam("focal_length", focal_length);
        nh_priv_.getParam("alpha_cam", alpha_cam);
        std::vector<float> principal_point_vector;
        nh_priv_.getParam("principal_point", principal_point_vector);
        principal_point << principal_point_vector[0], principal_point_vector[1];
        nh_priv_.getParam("gt_topic_name", gt_topic_name);
        nh_priv_.getParam("source_odom_name", source_odom_name);
        nh_priv_.getParam("target_odom_names", target_odom_names);

        nh_priv_.param<string>("root", root_, "/home/arpl/");
        nh_priv_.param<string>("output_file_name", output_file_name_, "jpdaf_output");



        ROS_INFO("===========================================");
        ROS_INFO("Pd: %f", pd);
        ROS_INFO("Gamma: %f", gamma);
        ROS_INFO("false measurements density: %0.9f", false_measurements_density);
        ROS_INFO("Alpha_0 threshold: %f", alpha_0_threshold);
        ROS_INFO("Beta_0 threshold: %f", beta_0_threshold);
        ROS_INFO("Max missed rate: %d", max_missed_rate);
        ROS_INFO("Min acceptance rate: %d", min_acceptance_rate);
        ROS_INFO("R: %f, %f", R_vector[0], R_vector[1]);
        ROS_INFO("T: %f, %f", T_vector[0], T_vector[1]);
        ROS_INFO("P_0: %f, %f, %f, %f", P_0_vector[0], P_0_vector[1], P_0_vector[2], P_0_vector[3]);
        ROS_INFO("Number of drones: %d", nb_drones);
        ROS_INFO("Max update time rate: %f", max_update_time_rate);
        ROS_INFO("Focal length: %f", focal_length);
        ROS_INFO("Alpha camera: %f", alpha_cam);
        ROS_INFO("Principal point: %f %f", principal_point(0), principal_point(1));
        ROS_INFO("Groundtruth source topic name:");
        cout << gt_topic_name << source_odom_name << endl;
        ROS_INFO("Groundtruth target topic names:");
        for(uint n=0; n<target_odom_names.size(); n++)
        {
            cout << gt_topic_name << target_odom_names[n] << endl;
        }

    }

}
