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
        track(false);
    }

}

void Node::detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes)
{
    bounding_boxes_msgs_buffer_.push_back(*bounding_boxes);
    if(image_buffer_.size() != 0)
    {
        track(true);
    }
}

void Node::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    image_buffer_.push_back(img_msg);
    if(bounding_boxes_msgs_buffer_.size() != 0)
    {
        track(true);
    }
}

void Node::poseCallback(const geometry_msgs::PoseStamped& pose_msg)
{
    pose_buffer_.push_back(pose_msg);
    ROS_INFO("Pushed back new pose");
}

/*void Node::gtCallback(const nav_msgs::OdometryConstPtr& msg)
{
    gt_odom_buffer_.push_back(*msg);
    ROS_INFO("Received new mocap msg");
    gt_odom_buffer_.clear();
}*/


void Node::track(bool called_from_detection)
{
    update_timer.stop();

    ROS_INFO("---------------------------------------------");
    if(!called_from_detection)
    {
        ROS_WARN("Tracking called from timer, no new detections");
    }
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
        //ROS_INFO("tracking called with time step %f, detection boxes nb: %d", time_step, (int)detections.size());
        if(time_step < 0)
        {
            ROS_ERROR("Negative time step! %f", time_step);
            time_step = 0;
        }

//        ROS_INFO("Real measured time step: %f", ros::Time::now().toSec() - last_timestamp_ros_time_now_debug);
        compute_timescaled_orientation_change_flush_pose((double)(last_timestamp + time_step));
        
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

        auto betas_matrix = compute_betas_matrix(hypothesis_mats, hypothesis_probs);
        cout << "betas_matrix: " << endl << betas_matrix << endl;


        std::vector<double> betas_0;
        for(uint t=0; t<tracks_.size(); t++)
        {
            std::vector<double> beta(betas_matrix.rows());
            double beta_0 = 1.0;
            for(int m=0; m<betas_matrix.rows(); m++)
            {
                beta[m] = betas_matrix(m, t+1);
                beta_0 -= betas_matrix(m, t+1);
            }
            betas_0.push_back(beta_0);
            tracks_[t].update(detections, beta, beta_0);
        }
        std::vector<double> alphas_0; for(int m=0; m<betas_matrix.rows(); m++){alphas_0.push_back(betas_matrix(m, 0));} // betas 0 are probabilities that track t has not been detected. Alphas 0 are probabilities that measurement m was generated by clutter noise. So beta 0 does NOT correspond to first column  of betas matrix
        //------------
        
        draw_tracks_publish_image(last_image);
        publishTracks();

        ROS_INFO("Calling manage new old tracks");
        manage_new_old_tracks(detections, alphas_0, betas_0);

    }
    if(called_from_detection)
    {
        last_timestamp = max(last_detection.header.stamp.toSec(), last_image->header.stamp.toSec());
    }
    else
    {
        last_timestamp += params.max_update_time_rate;
    }
//    last_timestamp_ros_time_now_debug = ros::Time::now().toSec();

    bounding_boxes_msgs_buffer_.clear();
    image_buffer_.clear();

    update_timer.start();
}


Eigen::MatrixXf Node::association_matrix(const std::vector<Detection> detections)
{
    Eigen::MatrixXf q(detections.size(), tracks_.size()+1);
    q.setZero();

    //Eigen::MatrixXf evaluation_mat(detections.size(), tracks_.size());
    //evaluation_mat.setZero();


    for(uint i=0; i<detections.size(); i++) {q(i,0)=1;} // Setting first column to 1
    
    for(uint i=0; i<detections.size(); i++)
    {
        for (uint j=0; j<tracks_.size(); j++)
        {
            Eigen::Vector2f measure = detections[i].getVect();
            Eigen::Vector2f prediction = tracks_[j].get_z_predict();
            if((measure-prediction).transpose() * tracks_[j].S().inverse() * (measure-prediction) <= pow(params.gamma, 2))
            {
                q(i, j+1)=1;
            }
            //evaluation_mat(i,j) = (measure-prediction).transpose() * tracks_[j].S().inverse() * (measure-prediction);
        }
    }
    //std::cout << "evaluation_mat computed: " << endl << evaluation_mat << endl;
    return q;
}

 
Eigen::MatrixXf Node::compute_betas_matrix(std::vector<Eigen::MatrixXf> hypothesis_mats, std::vector<double> hypothesis_probs)
{
    Eigen::MatrixXf betas_matrix(hypothesis_mats[0].rows(), hypothesis_mats[0].cols());
    betas_matrix.setZero();
    for(int i=0; i<(int)hypothesis_mats.size(); i++)
    {
        betas_matrix += hypothesis_probs[i]*hypothesis_mats[i];
    }
    return betas_matrix;
}

std::vector<Eigen::MatrixXf> Node::generate_hypothesis_matrices(Eigen::MatrixXf assoc_mat)
{
    std::vector<Eigen::MatrixXf> hypothesis_matrices;

    if(assoc_mat.rows() == 0)
    {
        Eigen::MatrixXf hypothesis(assoc_mat.rows(), assoc_mat.cols());
        hypothesis.setZero();
        hypothesis_matrices.push_back(hypothesis);
        return hypothesis_matrices;
    }
    
    std::vector<std::vector<int>> non_zero_indexes_per_row;
    for(int i=0; i<assoc_mat.rows(); i++)
    {
        non_zero_indexes_per_row.push_back(get_nonzero_indexes_row(assoc_mat.row(i)));
    }

    std::vector<int> row_iterators(assoc_mat.rows(), 0);
    
    bool finished = false;

    while(!finished)
    {
        //creating hypothesis matrix and pushing it if feasible
        Eigen::MatrixXf hypothesis(assoc_mat.rows(), assoc_mat.cols());
        hypothesis.setZero();
        for(int i=0; i<assoc_mat.rows(); i++)
        {
            hypothesis(i, (non_zero_indexes_per_row[i])[row_iterators[i]])=1;
        }

        Eigen::MatrixXf col_sum(1, hypothesis.cols());
        col_sum.setZero();
        for(int i=0; i < hypothesis.rows(); ++i)
        {
            col_sum += hypothesis.row(i);
        }
        
        bool feasible = true;
        for(int j=1;j<hypothesis.cols(); j++)
        {
            if(col_sum(0,j)>1)
            feasible = false;
        }
        if(feasible)
        {
            hypothesis_matrices.push_back(hypothesis);
        }

        //switching iterators for next hypothesis matrix
        row_iterators[0] = row_iterators[0]+1;
        for(int i=0; i<assoc_mat.rows(); i++)
        {
            if(row_iterators[i] == (int)non_zero_indexes_per_row[i].size())
            {
                if(i != assoc_mat.rows() -1)
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

std::vector<double> Node::compute_probabilities_of_hypothesis_matrices(std::vector<Eigen::MatrixXf> hypothesis_matrices, std::vector<Detection> detections)
{
    std::vector<double> probabilities;
    if(hypothesis_matrices[0].rows() == 0)
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
        ROS_ERROR("sum of probabilities is 0. This may mean the parameters are uncorrect.");
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

double Node::probability_of_hypothesis_unnormalized(Eigen::MatrixXf hypothesis, std::vector<Detection> detections)
{
    auto tau_ = tau(hypothesis);
    auto delta_ = delta(hypothesis);
    int nb_associated_measurements = (int)tau_.sum();
    int phi = tau_.rows() - nb_associated_measurements; // nb of false measurements
    int nb_associated_tracks = (int)delta_.sum();

    std::vector<Eigen::Vector2f> y_tilds;
    std::vector<Eigen::Matrix2f> Ss;

    for(int i=0; i<hypothesis.rows(); i++)
    {
        if(tau_(i, 0) == 1)
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
    double product_3 = pow((1-params.pd), hypothesis.cols() -1 - nb_associated_tracks);
    double probability = pow(params.false_measurements_density, phi) * product_1 * product_2 * product_3;

    return probability;
}

Eigen::MatrixXf Node::tau(Eigen::MatrixXf hypothesis)
{
    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
    Eigen::MatrixXf row_sum(hypothesis.rows(), 1);
    row_sum.setZero();
    for(int i=1; i < hypothesis.cols(); ++i)
    {
        row_sum += hypothesis.col(i);
    }
    return row_sum;
}

Eigen::MatrixXf Node::delta(Eigen::MatrixXf hypothesis)
{
    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
    Eigen::MatrixXf col_sum(1, hypothesis.cols());
    col_sum.setZero();
    for(int i=0; i < hypothesis.rows(); ++i)
    {
        col_sum += hypothesis.row(i);
    }
    Eigen::MatrixXf delta(1, hypothesis.cols()-1);
    delta.setZero(); 
    delta = col_sum.block(0, 1, 1, (int)(hypothesis.cols()-1));
    return delta;
}

void Node::manage_new_old_tracks(std::vector<Detection> detections, std::vector<double> alphas_0, std::vector<double> betas_0)
{
    ROS_INFO("manage new old tracks: alphas_0 length=%d, betas_0 length=%d", (int)alphas_0.size(), (int)betas_0.size());
    for(uint i=0; i<tracks_.size(); i++)
    {
        tracks_[i].increase_lifetime();
    }

    std::vector<int> unassoc_detections_idx;

    for(uint i=0; i<alphas_0.size(); i++)
    {
        if(alphas_0[i] >= params.alpha_0_threshold)
        {
            unassoc_detections_idx.push_back((int)i);
        }
    }

    auto new_tracks = create_new_tracks(detections, unassoc_detections_idx);

    for(uint j=0; j<betas_0.size(); j++)
    {
        if(betas_0[j] >= params.beta_0_threshold)
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
        Eigen::MatrixXf costMat(prev_unassoc_size, unassoc_size);
        std::vector<float> costs(unassoc_size * prev_unassoc_size);
        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            for(uint j = 0; j < unassoc_size; ++j)
            {
                auto tmp = Eigen::Vector2f(unassoc_detections[j].x()-prev_unassoc_detections[i].x(), unassoc_detections[j].y()-prev_unassoc_detections[i].y());
	            costs.at(i + j * prev_unassoc_size ) = tmp.norm();
                costMat(i, j) = costs.at(i + j*prev_unassoc_size);
            }
        }
            
        std::vector<int> assignments;
        AssignmentProblemSolver APS;
        APS.Solve(costs, prev_unassoc_size, unassoc_size, assignments, AssignmentProblemSolver::optimal);
        //returned assignments is of length previous unassigned

        const uint& assSize = assignments.size();
        Eigen::MatrixXf assigmentsBin(prev_unassoc_size, unassoc_size);
        assigmentsBin.setZero();
        for(uint i = 0; i < assSize; ++i)
        {
            if( assignments[i] != -1 && costMat(i, assignments[i]) < params.assoc_cost)
            {
            	assigmentsBin(i, assignments[i]) = 1;
            }
        }

        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            for(uint j = 0; j < unassoc_size; ++j)
            {
                if(assigmentsBin(i, j))
                {
                    const float& vx = unassoc_detections.at(j).x() - prev_unassoc_detections.at(i).x();
                    const float& vy = unassoc_detections.at(j).y() - prev_unassoc_detections.at(i).y();
                    Track tr(unassoc_detections.at(j).x(), unassoc_detections.at(j).y(), vx, vy, params);
                    new_tracks.push_back(tr);
                    ROS_INFO("created new track!");
                }
            }
        }
    
        Eigen::MatrixXf sumAssoc(1, unassoc_size);
        sumAssoc.setZero();
        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            sumAssoc += assigmentsBin.row(i);
        }

        prev_unassoc_detections.clear();
        for(uint i=0; i<unassoc_size; i++)
        {
            if(sumAssoc(0, i) == 0)
            {
                prev_unassoc_detections.push_back(unassoc_detections.at(i));                
            }
        }
        return new_tracks;
    }
}

void Node::compute_timescaled_orientation_change_flush_pose(double detection_time_stamp)
{
    ROS_INFO("Pose buffer length: %d", (int)pose_buffer_.size());
    if((int)pose_buffer_.size() > 50)
    {
        ROS_FATAL("pose buffer is too long, there is a problem somewhere");
        exit(1);
    }
    cout << "Detection timestamp: " << detection_time_stamp << endl;
    cout << "Pose buffer timestamps: " ; for(int i=0; i<(int)pose_buffer_.size(); i++){cout << pose_buffer_[i].header.stamp.toSec() << ", ";} cout << endl;
    if((int)pose_buffer_.size() == 0)
    {
        ROS_ERROR("Pose buffer length is 0. Assuming no orientation change");
        return; //TODO return 0 
    }
    else if((int)pose_buffer_.size() == 1)
    {
        ROS_WARN("Pose buffer length is 1. Assuming no orientation change");
        return; //TODO return 0 
    }
    else if(detection_time_stamp - pose_buffer_.back().header.stamp.toSec() >= params.max_update_time_rate)
    {
        ROS_WARN("The pose buffer is running too late compaired to the detections. Assuming no orientation change");
        return; //TODO return 0     
    }
    else if(pose_buffer_.front().header.stamp.toSec() - detection_time_stamp >= 0)
    {
        ROS_WARN("The pose buffer doesn't contain elements prior to the detections. Assuming no orientation change");
        return; //TODO return 0     
    }

    int pose_buffer_index = 0;
    while(pose_buffer_index != (int)pose_buffer_.size())
    {
        if((double)pose_buffer_[pose_buffer_index].header.stamp.toSec() >= detection_time_stamp)
        {
            break;
        }
        pose_buffer_index++;
    }
    if(pose_buffer_index == (int)pose_buffer_.size())
    {
        ROS_WARN("Pose buffer is running late compaired to the detections.");
    }
    ROS_INFO("Pose buffer index: %d", pose_buffer_index);
    //Compute orientation shift rotation
    //Do not forget that pose_buffer_orientation can == pose_buffer_.size() if it has no advance!
    if(pose_buffer_index != (int)pose_buffer_.size())
    {
//stopped here!
    }
    else
    {

    }
    Eigen::Quaterniond orientation;
    //Compute yaw, pitch, roll

    //Scale yaw, pitch and roll to detection timestep

    std::vector<geometry_msgs::PoseStamped> temp;
    for(int i=pose_buffer_index; i<(int)pose_buffer_.size(); i++)
    {
        temp.push_back(pose_buffer_[i]);
    }
    pose_buffer_.clear();
    pose_buffer_ = temp;
    //return scaled yaw, pitch and roll


}

void Node::draw_tracks_publish_image(const sensor_msgs::ImageConstPtr last_image)
{
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


std::vector<int> Node::get_nonzero_indexes_row(Eigen::MatrixXf mat)
{
    std::vector<int> nonzero_elements;
    if (mat.rows() != 1)
    {
        ROS_ERROR("get_nonzero_elements_row called, argument not row!");
        return nonzero_elements;
    }
    for (int i=0; i<mat.cols(); i++)
    {
        if(mat(0,i) != 0)
        {
            nonzero_elements.push_back(i);
        }    
    }
    return nonzero_elements;
}



}
