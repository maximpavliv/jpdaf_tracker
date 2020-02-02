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

        for(auto track : tracks_)
        {
            track.predict(time_step);
        }

        auto assoc_mat = association_matrix(detections);
        std::cout << "assoc_mat: " << endl << assoc_mat << endl;

        std::vector<cv::Mat_<int>> hypothesis_mats = generate_hypothesis_matrices(assoc_mat);
/*        cout << "hypothesis matrices: " << endl;
        for(auto hyp : hypothesis_mats)
        {
            cout << hyp << endl << endl;
        }*/

        for(auto track : tracks_)
        {
            track.gainUpdate();
        }


        for(auto track : tracks_)
        {
            std::vector<float> beta(detections.size(), 0.0);
            float beta_0 = 1;
            track.update(detections, beta, beta_0);
        }

        auto not_assoc_dets = not_associated_detections(assoc_mat);
        ROS_INFO("number of not associated detections: %d", (int)not_assoc_dets.size());
        
        manage_new_tracks(detections, not_assoc_dets);
        manage_old_tracks();
        validate_tracks();

    }

    last_timestamp = last_detection.header.stamp.toSec();
    bounding_boxes_msgs_buffer_.clear();
    image_buffer_.clear();
}


cv::Mat_<int> Node::association_matrix(const std::vector<Detection> detections)
{
    cv::Mat_<int> q(cv::Size(tracks_.size()+1, detections.size()), int(0));

    cv::Mat_<float> evaluation_mat(cv::Size(tracks_.size(), detections.size()), float(0));

    for(uint i=0; i<detections.size(); i++) {q.at<int>(i,0)=1;} // Setting first column to 1
    
    for(uint i=0; i<detections.size(); i++)
    {
        for (uint j=0; j<tracks_.size(); j++)
        {
            Eigen::Vector2f measure = detections[i].getVect();
            Eigen::Vector2f prediction = tracks_[j].get_z_predict();
            if((measure-prediction).transpose() * tracks_[j].S().inverse() * (measure-prediction) <= pow(params.gamma, 2))
            {
                q.at<int>(i, j+1)=1;
            }

            evaluation_mat.at<float>(i,j) = (measure-prediction).transpose() * tracks_[j].S().inverse() * (measure-prediction);
        }
    }
    std::cout << "evaluation_mat computed: " << endl << evaluation_mat << endl;
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

std::vector<cv::Mat_<int>> Node::generate_hypothesis_matrices(cv::Mat_<int> assoc_mat)
{
    std::vector<cv::Mat_<int>> hypothesis_matrices;
    
    std::vector<std::vector<int>> non_zero_indexes_per_row;
    for(int i=0; i<assoc_mat.rows; i++)
    {
        non_zero_indexes_per_row.push_back(get_nonzero_indexes_row(assoc_mat.row(i)));
    }

    std::vector<int> row_iterators(assoc_mat.rows, 0);
    
    bool finished = false;

    while(!finished)
    {
        //creating hypothesis matrix and pushing it if feasible
        cv::Mat_<int> hypothesis(cv::Size(assoc_mat.cols, assoc_mat.rows), int(0));
        for(int i=0; i<assoc_mat.rows; i++)
        {
            hypothesis.at<int>(i, (non_zero_indexes_per_row[i])[row_iterators[i]])=1;
        }

        cv::Mat col_sum(cv::Size(hypothesis.cols, 1), hypothesis.type(), cv::Scalar(0));
        for(int i=0; i < hypothesis.rows; ++i)
        {
            col_sum += hypothesis.row(i);
        }
        
        bool feasible = true;
        for(int j=1;j<hypothesis.cols; j++)
        {
            if(col_sum.at<int>(0,j)>1)
            feasible = false;
        }
        if(feasible)
        {
            hypothesis_matrices.push_back(hypothesis);
        }

        //switching iterators for next hypothesis matrix
        row_iterators[0] = row_iterators[0]+1;
        for(int i=0; i<assoc_mat.rows; i++)
        {
            if(row_iterators[i] == (int)non_zero_indexes_per_row[i].size())
            {
                if(i != assoc_mat.rows -1)
                {
                    row_iterators[i] = 0;
                    row_iterators[i+1]++;
                }
                else
                {
                    finished = true;
                }
            }
        }
    }

    return hypothesis_matrices;
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


void Node::validate_tracks()
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


std::vector<int> Node::get_nonzero_indexes_row(cv::Mat_<int> mat)
{
    std::vector<int> nonzero_elements;
    if (mat.rows != 1)
    {
        ROS_ERROR("get_nonzero_elements_row called, argument not row!");
        return nonzero_elements;
    }
    for (int i=0; i<mat.cols; i++)
    {
        if(mat.at<int>(0,i) != 0)
        {
            nonzero_elements.push_back(i);
        }    
    }
    return nonzero_elements;
}



}
