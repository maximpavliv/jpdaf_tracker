#include<jpdaf_tracker/node.h>

using namespace std;

namespace jpdaf {


Node::Node(ros::NodeHandle nh, ros::NodeHandle nh_priv):
    nh_(nh),
    nh_priv_(nh_priv)
{
    TrackerParam par(nh_priv_);
    params = par;

/*    K_ << intrinsics_[0], 0., intrinsics_[2],
                                   0., intrinsics_[1], intrinsics_[3],
                                   0., 0., 1.f;*/


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
    ROS_INFO("---------------------------------------------");
    auto last_image = image_buffer_.back();
    auto last_detection = bounding_boxes_msgs_buffer_.back();
    //int number_detections = last_detection.bounding_boxes.size();
    auto detections = get_detections(last_detection);


    if(track_init)
    {
        for (auto det : detections)
        {
            prev_unassoc_detections.push_back(det);
        }
        track_init = false;
    }
    else
    {
        double time_step = last_detection.header.stamp.toSec() - last_timestamp;
        ROS_INFO("tracking called with time step %f, detection boxes nb: %d", time_step, (int)detections.size());

        auto assoc_mat = association_matrix(detections);
        std::cout << "assoc_mat: " << endl << assoc_mat << endl;
        auto not_assoc_dets = not_associated_detections(assoc_mat);
        ROS_INFO("number of not associated detections: %d", (int)not_assoc_dets.size());
        
        manage_new_tracks(detections, not_assoc_dets);
        //TODO WARNING: nb of tracks can be different than nb of columns in assoc_matrix - 1 !! (Since potentially new Tracks). Handle this possibility!!

    }

    last_timestamp = last_detection.header.stamp.toSec();
    bounding_boxes_msgs_buffer_.clear();
    image_buffer_.clear();
}


cv::Mat_<int> Node::association_matrix(const std::vector<Detection> detections)
{
    cv::Mat_<int> q(cv::Size(tracks_.size()+1, detections.size()), int(0));
    for(uint i=0; i<detections.size(); i++) {q.at<int>(i,0)=1;} // Setting first column to 1

    for(uint i=0; i<detections.size(); i++)
    {
        for (uint j=0; j<tracks_.size(); j++)
        {
            Eigen::Vector2f measure = detections[i].getVect();
            Eigen::Vector2f prediction = tracks_[j].get_z_predict();
            if((measure-prediction).transpose() * tracks_[j].S().inverse() * (measure-prediction) <= params.gamma)
                q.at<int>(i, j+1)=1;
        }
    }
    return q;
}

std::vector<int> Node::not_associated_detections(cv::Mat_<int> assoc_mat)
{
    cv::Mat row_sum(cv::Size(1, assoc_mat.rows), assoc_mat.type(), cv::Scalar(-1));
    for(int i=0; i < assoc_mat.cols; ++i)
    {
        row_sum += assoc_mat.col(i);
    }
    std::vector<int> not_associated_detections;
    for(int j=0; j<assoc_mat.rows; j++)
    {
        if(row_sum.at<int>(j,0)==0)
            not_associated_detections.push_back(j);
    }
    return not_associated_detections;
}

void Node::manage_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections_idx)
{
    const uint& prev_unassoc_size = prev_unassoc_detections.size();
    const uint& unassoc_size = unassoc_detections_idx.size();
    std::vector<Detection> unassoc_detections;
    for(auto idx : unassoc_detections_idx)
    {
        unassoc_detections.push_back(detections[idx]);
    }

    if(prev_unassoc_size == 0)
    {
        prev_unassoc_detections.clear(); //just to be sure
        prev_unassoc_detections = unassoc_detections;
        return;
    }
    else if (unassoc_size == 0)
    {
        prev_unassoc_detections.clear();
        return;
    }
    else
    {
        cv::Mat costMat = cv::Mat(cv::Size(unassoc_size, prev_unassoc_size), CV_32FC1);
        std::vector<float> costs(unassoc_size * prev_unassoc_size);
        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            for(uint j = 0; j < unassoc_size; ++j)
            {
                auto tmp = Eigen::Vector2f(unassoc_detections[j].x()-prev_unassoc_detections[i].x(), unassoc_detections[j].y()-prev_unassoc_detections[i].y());
	            costs.at(i + j * prev_unassoc_size ) = tmp.norm();
                costMat.at<float>(i, j) = costs.at(i + j*prev_unassoc_size);
            }
        }
            
        std::vector<int> assignments;
        AssignmentProblemSolver APS;
        APS.Solve(costs, prev_unassoc_size, unassoc_size, assignments, AssignmentProblemSolver::optimal);
        //returned assignments is of length previous unassigned

        const uint& assSize = assignments.size();
        cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(unassoc_size, prev_unassoc_size), CV_32SC1);
        for(uint i = 0; i < assSize; ++i)
        {
            if( assignments[i] != -1 && costMat.at<float>(i, assignments[i]) < params.assoc_cost)
            {
            	assigmentsBin.at<int>(i, assignments[i]) = 1;
            }
        }

        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            for(uint j = 0; j < unassoc_size; ++j)
            {
                if(assigmentsBin.at<int>(i, j))
                {
                    const float& vx = unassoc_detections.at(j).x() - prev_unassoc_detections.at(i).x();
                    const float& vy = unassoc_detections.at(j).y() - prev_unassoc_detections.at(i).y();
                    Track tr(unassoc_detections.at(j).x(), unassoc_detections.at(j).y(), vx, vy, params);
                    tracks_.push_back(tr);
                }
            }
        }
    
        cv::Mat sumAssoc(cv::Size(unassoc_size, 1), CV_32SC1, cv::Scalar(0));
        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            sumAssoc += assigmentsBin.row(i);
        }

        prev_unassoc_detections.clear();
        for(uint i=0; i<unassoc_size; i++)
        {
            if(sumAssoc.at<int>(0, i) == 0)
            {
                prev_unassoc_detections.push_back(unassoc_detections.at(i));                
            }
        }
        return;
    }
}


void Node::validate_new_tracks()
{
    for(auto track : tracks_)
    {
        if(track.getId() == -1 && track.isValidated())
        {
            if(!(lost_tracks.empty()))
            {
                track.setId(lost_tracks[0]);
                lost_tracks.erase(lost_tracks.begin());
            }
        }
    }
}

void Node::manage_old_tracks()
{
    std::vector<Track> tmp;
    for(uint i=0; i<tracks_.size(); i++)
    {
        if(!(tracks_[i].isDeprecated()))
        {            
            tmp.push_back(tracks_[i]);
        }
        else
        {
            if(tracks_[i].getId() != -1)
                lost_tracks.push_back(tracks_[i].getId());
        }
    }
    tracks_.clear();
    tracks_ = tmp;
}

std::vector<Detection> Node::get_detections(const darknet_ros_msgs::BoundingBoxes last_detection)
{
    std::vector<Detection> norm_det;
    for(uint i=0; i<last_detection.bounding_boxes.size(); i++)\
    {
        Detection one_det(float(last_detection.bounding_boxes[i].xmin+last_detection.bounding_boxes[i].xmax)/2, 
                          float(last_detection.bounding_boxes[i].ymin+last_detection.bounding_boxes[i].ymax)/2, 
                          last_detection.bounding_boxes[i].xmax-last_detection.bounding_boxes[i].xmin, 
                          last_detection.bounding_boxes[i].ymax-last_detection.bounding_boxes[i].ymin);
        norm_det.push_back(one_det);
    }
    return norm_det;
}


}
