#include<jpdaf_tracker/node.h>

using namespace std;

namespace jpdaf {


Node::Node(ros::NodeHandle nh, ros::NodeHandle nh_priv):
    nh_(nh),
    nh_priv_(nh_priv),
    it_(nh_priv)
{
    TrackerParam par(nh_priv_);
    params = par;



    detection_sub_ = nh_priv_.subscribe("detection", 10, &Node::detectionCallback, this);
    image_sub_ = nh_priv_.subscribe("image", 10, &Node::imageCallback, this);
    pose_sub_ = nh_priv_.subscribe("pose", 10, &Node::poseCallback, this);
    //mocap_sub_ = nh_priv_.subscribe("gt", 10, &Node::gtCallback, this);

    update_timer = nh.createTimer(ros::Duration(params.max_update_time_rate), &Node::timer_callback, this);

    image_pub_ = it_.advertise("image_tracked", 1);
    tracks_pub_ = nh.advertise<jpdaf_tracker_msgs::Tracks>("jpdaf_tracks", 1);

    track_init = true;
    
    for(int i=1; i<=params.nb_drones; i++)
    {
        lost_tracks.push_back(i);
    }

    ROS_INFO("Node initialized successfully");
}

void Node::timer_callback(const ros::TimerEvent& event)
{
    if(image_buffer_.size() != 0)
    {
        ROS_WARN("Calling track without detections");
        track(false);
    }

}

void Node::detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes)
{
    bounding_boxes_msgs_buffer_.push_back(*bounding_boxes);
    if(image_buffer_.size() != 0)
    {
        update_timer.stop();
        track(true);
        update_timer.start();
    }
}

void Node::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    image_buffer_.push_back(img_msg);
    if(bounding_boxes_msgs_buffer_.size() != 0)
    {    update_timer.stop();
        track(true);
        update_timer.start();
    }
}

void Node::poseCallback(const geometry_msgs::PoseStamped& pose_msg)
{
    pose_buffer_.push_back(pose_msg);
}

/*void Node::gtCallback(const nav_msgs::OdometryConstPtr& msg)
{
    gt_odom_buffer_.push_back(*msg);
    ROS_INFO("Received new mocap msg");
    gt_odom_buffer_.clear();
}*/


void Node::track(bool called_from_detection)
{
    ROS_INFO("---------------------------------------------");
    auto last_image = image_buffer_.back();

    darknet_ros_msgs::BoundingBoxes last_detection;
    if(called_from_detection)
    {
        last_detection = bounding_boxes_msgs_buffer_.back();
    }
    else
    {
        std::vector<darknet_ros_msgs::BoundingBox> emptyVector;
        last_detection.bounding_boxes = emptyVector;
    }
    auto detections = get_detections(last_detection);


    if(track_init)
    {
        for (uint d=0; d<detections.size(); d++)
        {
            prev_unassoc_detections.push_back(detections[d]);
        }
        track_init = false;
    }
    else
    {
        cout << "Currently tracked tracks indexes: " << endl; for(auto tr : tracks_){cout << tr.getId() << " ";} cout << endl;
        double time_step;
        if(called_from_detection)
        {
            time_step = max(last_detection.header.stamp.toSec(), last_image->header.stamp.toSec()) - last_timestamp;
        }
        else
        {
            time_step = params.max_update_time_rate;
        }
        ROS_INFO("tracking called with time step %f, detection boxes nb: %d", time_step, (int)detections.size());

        compute_timescaled_orientation_shift_flush_pose();
        
        //PREDICTION
        for(uint t=0; t<tracks_.size(); t++)
        {
            tracks_[t].predict(time_step);
        }
        //------------

        //COMPUTE GAIN
        for(uint t=0; t<tracks_.size(); t++)
        {
            tracks_[t].gainUpdate();
        }
        //------------


        //UPDATE
        auto assoc_mat = association_matrix(detections);
        std::cout << "assoc_mat: " << endl << assoc_mat << endl;

        auto hypothesis_mats = generate_hypothesis_matrices(assoc_mat);
        auto hypothesis_probs = compute_probabilities_of_hypothesis_matrices(hypothesis_mats, detections);
        ROS_INFO("Nb of hypotheses: %d", (int)hypothesis_mats.size());

        /*cout << "hypothesis matrices and their respective probabilities:" << endl;
        for(uint h=0; h<hypothesis_mats.size(); h++)
        {
            cout << hypothesis_mats[h] << endl << "prob: " <<hypothesis_probs[h] << endl << endl;
        }*/

        std::vector<double> betas_0;//beta_0 of each track, used to determine if track has not been detected well

        for(uint t=0; t<tracks_.size(); t++)
        {
            std::vector<double> beta = compute_beta(t, hypothesis_mats, hypothesis_probs);
            cout << "track " << tracks_[t].getId() << " betas: "; for(uint b=0; b<beta.size(); b++){cout << beta[b] << " ";} cout << endl;
            double sum_betas = 0; 
            for(uint b=0; b<beta.size(); b++)
            {
                sum_betas += beta[b];
            }
            double beta_0 = 1 - sum_betas;
            cout << "beta_0: " << beta_0 << endl;
            cout << "updating track with ID " << tracks_[t].getId() << endl;
            tracks_[t].update(detections, beta, beta_0);
            betas_0.push_back(beta_0);
        }

        //------------
        
        draw_tracks_publish_image(last_image);
        publishTracks();

        //-------------
        auto alphas_0 = compute_alphas_0(hypothesis_mats, hypothesis_probs);
        auto betas_0_bis = compute_betas_0(hypothesis_mats, hypothesis_probs);

        manage_new_old_tracks(detections, betas_0_bis, alphas_0);

    }
    if(called_from_detection)
        last_timestamp = max(last_detection.header.stamp.toSec(), last_image->header.stamp.toSec());
    else
        last_timestamp += params.max_update_time_rate;
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


std::vector<double> Node::compute_beta(int track_nb, std::vector<cv::Mat_<int>> hypothesis_matrices, std::vector<double> hypothesis_probabilities)
{
    std::vector<double> beta;
    for(int i=0; i<hypothesis_matrices[0].rows; i++) // for each measurement
    {
        double beta_i = 0;
        for(uint h=0; h<hypothesis_matrices.size(); h++) // for each hypothesis
        {
            if(hypothesis_matrices[h].at<int>(i, track_nb + 1) == 1)
            {
                beta_i += hypothesis_probabilities[h];
            }
        }
        beta.push_back(beta_i);
    }
    return beta;
}

std::vector<double> Node::compute_alphas_0(std::vector<cv::Mat_<int>> hypothesis_mats, std::vector<double> hypothesis_probs)
{
    std::vector<double> alphas_0(hypothesis_mats[0].cols - 1, 0.0);

    for(uint h=0; h<hypothesis_mats.size(); h++)
    {
        auto detected_tracks = delta(hypothesis_mats[h]);
        for(int i=1; i<hypothesis_mats[0].cols; i++)
        {
            if(detected_tracks.at<int>(i-1) == 0)
            {
                alphas_0[i-1] += hypothesis_probs[h];
            }
        }
    }
    
    return alphas_0;
}

std::vector<double> Node::compute_betas_0(std::vector<cv::Mat_<int>> hypothesis_mats, std::vector<double> hypothesis_probs)
{
    std::vector<double> betas_0(hypothesis_mats[0].rows, 0.0);

    for(uint h=0; h<hypothesis_mats.size(); h++)
    {
        for(int i=0; i<hypothesis_mats[0].rows; i++)
        {
            if(hypothesis_mats[h].at<int>(i, 0) == 1)
            {
                betas_0[i] += hypothesis_probs[h];
            }
        }
    }
    
    return betas_0;
}

 
std::vector<cv::Mat_<int>> Node::generate_hypothesis_matrices(cv::Mat_<int> assoc_mat)
{
    std::vector<cv::Mat_<int>> hypothesis_matrices;

    if(assoc_mat.rows == 0)
    {
        cv::Mat_<int> hypothesis(cv::Size(assoc_mat.cols, assoc_mat.rows), int(0));
        hypothesis_matrices.push_back(hypothesis);
        return hypothesis_matrices;
    }
    
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

std::vector<double> Node::compute_probabilities_of_hypothesis_matrices(std::vector<cv::Mat_<int>> hypothesis_matrices, std::vector<Detection> detections)
{
    std::vector<double> probabilities;
    if(hypothesis_matrices[0].rows == 0)
    {
        double prob = 1;
        probabilities.push_back(prob);
        return probabilities;
    }
    for(uint h=0; h<hypothesis_matrices.size(); h++)
    {
        auto prob = probability_of_hypothesis_unnormalized(hypothesis_matrices[h], detections);
        probabilities.push_back(prob);
    }
    //Normalization:
    double sum = 0;
    for(uint p=0; p<probabilities.size(); p++)
    {
        sum += probabilities[p];
    }
    if(sum == 0)
    {
        ROS_WARN("sum of probabilities is 0. This may mean the parameters are uncorrect.");
        for(uint i=0; i<probabilities.size(); i++)
        {
            probabilities[i] = 1/probabilities.size();
        }
    }
    else
    {
        for(uint i=0; i<probabilities.size(); i++)
        {
            probabilities[i] /= sum;
        }
    }
    return probabilities;
}

double Node::probability_of_hypothesis_unnormalized(cv::Mat_<int> hypothesis, std::vector<Detection> detections)
{
    auto tau_ = tau(hypothesis);
    auto delta_ = delta(hypothesis);
    int nb_associated_measurements = (int)((float)sum(tau_)[0]);//one channel
    int phi = tau_.rows - nb_associated_measurements; // nb of false measurements
    int nb_associated_tracks = (int)((float)sum(delta_)[0]);//one channel

    std::vector<Eigen::Vector2f> y_tilds;
    std::vector<Eigen::Matrix2f> Ss;

    for(int i=0; i<hypothesis.rows; i++)
    {
        if(tau_.at<int>(i, 0) == 1)
        {
            int measurement_index = i;
            int track_index;
            auto track_indexes = get_nonzero_indexes_row(hypothesis.row(i));
            if(track_indexes.size() != 1)
                ROS_ERROR("hypothesis matrix uncorrect, multiple sources for same measure! If this happens there is an error in the code");
            track_index = track_indexes[0] - 1;
            y_tilds.push_back(detections[measurement_index].getVect() - tracks_[track_index].get_z_predict());
            Ss.push_back(tracks_[track_index].S());
        }

    }

    int M = 2;//dimensionality of observations

    double product_1 = 1;
    for(int i=0; i< nb_associated_measurements; i++)
    {
        product_1 *= (exp(-(double)(y_tilds[i].transpose()*Ss[i].inverse()*y_tilds[i])/2)) / (sqrt(pow(2*M_PI, M) * Ss[i].determinant()));
        
    }
    double product_2 = pow(params.pd, nb_associated_tracks);
    double product_3 = pow((1-params.pd), hypothesis.cols -1 - nb_associated_tracks);
    double probability = pow(params.false_measurements_density, phi) * product_1 * product_2 * product_3;

    return probability;
}

cv::Mat_<int> Node::tau(cv::Mat_<int> hypothesis)
{
    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
    cv::Mat row_sum(cv::Size(1, hypothesis.rows), hypothesis.type(), cv::Scalar(0));
    for(int i=1; i < hypothesis.cols; ++i)
    {
        row_sum += hypothesis.col(i);
    }
    return row_sum;
}

cv::Mat_<int> Node::delta(cv::Mat_<int> hypothesis)
{
    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
    cv::Mat col_sum(cv::Size(hypothesis.cols, 1), hypothesis.type(), cv::Scalar(0));
    for(int i=0; i < hypothesis.rows; ++i)
    {
        col_sum += hypothesis.row(i);
    }
    cv::Mat delta(cv::Size(hypothesis.cols-1, 1), hypothesis.type(), cv::Scalar(0));
    cv::Rect roi(1, 0, hypothesis.cols-1, 1);    
    delta = col_sum(roi);
    return delta;
}

void Node::manage_new_old_tracks(std::vector<Detection> detections, std::vector<double> betas_0, std::vector<double> alphas_0)
{
    for(uint i=0; i<tracks_.size(); i++)
    {
        tracks_[i].increase_lifetime();
    }

    std::vector<int> unassoc_detections_idx;

    for(uint i=0; i<betas_0.size(); i++)
    {
        if(betas_0[i] >= params.beta_0_threshold)
        {
            unassoc_detections_idx.push_back((int)i);
        }
    }

    auto new_tracks = create_new_tracks(detections, unassoc_detections_idx);\
    for(uint j=0; j<alphas_0.size(); j++)
    {
        if(alphas_0[j] >= params.alpha_0_threshold)
        {
            tracks_[j].has_not_been_detected();
        }
        else
        {
            tracks_[j].has_been_detected();
        }
    }

    std::vector<Track> tmp;
    for(uint t=0; t<tracks_.size(); t++)
    {
        if(!tracks_[t].isDeprecated())
        {
            tmp.push_back(tracks_[t]);
        }
        else
        {
            if(tracks_[t].getId() != -1)
            {
                lost_tracks.push_back(tracks_[t].getId());
            }
            ROS_INFO("deleted track!");
        }
    }

    for(uint t=0; t<new_tracks.size(); t++)
    {
        tmp.push_back(new_tracks[t]);
    }

    tracks_.clear();
    tracks_ = tmp;

    for(uint t=0; t<tracks_.size(); t++)
    {
        if(tracks_[t].getId() == -1 && tracks_[t].isValidated())
        {
            if(!lost_tracks.empty())
            {
                tracks_[t].setId(lost_tracks[0]);
                lost_tracks.erase(lost_tracks.begin());
            }
        }
    }
}

std::vector<Track> Node::create_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections_idx)
{
    std::vector<Track> new_tracks;

    const uint& prev_unassoc_size = prev_unassoc_detections.size();
    const uint& unassoc_size = unassoc_detections_idx.size();
    std::vector<Detection> unassoc_detections;

    for(uint i=0; i<unassoc_detections_idx.size(); i++)
    {
        unassoc_detections.push_back(detections[unassoc_detections_idx[i]]);
    }

    if(prev_unassoc_size == 0)
    {
        prev_unassoc_detections.clear(); //just to be sure
        prev_unassoc_detections = unassoc_detections;
        return new_tracks;
    }
    else if (unassoc_size == 0)
    {
        prev_unassoc_detections.clear();
        return new_tracks;
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
                    new_tracks.push_back(tr);
                    ROS_INFO("created new track!");
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
        return new_tracks;
    }
}

void Node::compute_timescaled_orientation_shift_flush_pose(void)
{
    if((int)pose_buffer_.size() == 0)
    {
        ROS_ERROR("Pose buffer length is 0. Assuming no orientation shift");
        return; //TODO return 0 
    }
    if((int)pose_buffer_.size() == 1)
    {
        ROS_ERROR("Pose buffer length is 1. Assuming no orientation shift");
        return; //TODO return 0 
    }

    //TODO check buffer size checks and adapt if too strict!!!


    //Find pose with closest and smaller timestamp  to current timestamp
    //Find previous pose

    //Compute orientation shift rotation
    
    //Compute yaw, pitch, roll

    //Scale yaw, pitch and roll to detection timestep

    //flush buffer up to previous pose

    //return scaled yaw, pitch and roll

    pose_buffer_.clear();
}

void Node::draw_tracks_publish_image(const sensor_msgs::ImageConstPtr last_image)
{
    //cv_bridge::CvImage image = cv_bridge::toCvShare(last_image, "rgb8");

    cv_bridge::CvImageConstPtr im_ptr_ = cv_bridge::toCvShare(last_image, "rgb8");
    cv::Mat im = im_ptr_->image;

    if(im.empty()) return;

    for(uint t=0; t<tracks_.size(); t++)
    {
        if(tracks_[t].getId() != -1)
        {
            cv::Point2f tr_pos((int)(tracks_[t].get_z_update())(0), (int)(tracks_[t].get_z_update())(1));
            cv::Point2f id_pos(tr_pos.x, tr_pos.y+30);
            cv::circle(im, tr_pos, 4, cv::Scalar(0, 255, 255), 2);
            putText(im, to_string(tracks_[t].getId()), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cvScalar(0, 255, 255), 1, CV_AA);
        }
        else
        {
            cv::Point2f tr_pos((int)(tracks_[t].get_z_update())(0), (int)(tracks_[t].get_z_update())(1));
            cv::Point2f id_pos(tr_pos.x, tr_pos.y+30);
            cv::circle(im, tr_pos, 4, cv::Scalar(0, 0, 255), 2);
            putText(im, "-", id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cvScalar(0, 0, 255), 1, CV_AA);
        }
    }
    
    cv_bridge::CvImage processed_image_bridge;
    processed_image_bridge.header.stamp = last_image->header.stamp;
    processed_image_bridge.image = im;
    processed_image_bridge.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::ImagePtr im_msg = processed_image_bridge.toImageMsg();
    image_pub_.publish(im_msg);


    return;
}

void Node::publishTracks()
{
    jpdaf_tracker_msgs::Tracks trs_msg;
    for(uint t=0; t<tracks_.size(); t++)
    {
        if(tracks_[t].getId() != -1)
        {
            jpdaf_tracker_msgs::Track tr_msg;
            tr_msg.id = tracks_[t].getId();
            tr_msg.x = (int)(tracks_[t].get_z_update())(0);
            tr_msg.y = (int)(tracks_[t].get_z_update())(1);
            
            trs_msg.tracks.push_back(tr_msg);
        }
    }
    //add timestamp to header!!
    
    tracks_pub_.publish(trs_msg);
}


std::vector<Detection> Node::get_detections(const darknet_ros_msgs::BoundingBoxes last_detection)
{
    std::vector<Detection> norm_det;
    for(uint i=0; i<last_detection.bounding_boxes.size(); i++)
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
