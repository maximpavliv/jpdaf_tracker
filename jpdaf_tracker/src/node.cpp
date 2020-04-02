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
    imu_sub_ = nh_priv_.subscribe("imu", 10, &Node::imuCallback, this);
/*    source_odom_sub_ = nh_priv_.subscribe(params.gt_topic_name+params.source_odom_name, 10, &Node::GTSourceCallback, this);
    for(uint t=0; t<params.target_odom_names.size(); t++)
    {
        target_odom_subs_.push_back(nh_priv_.subscribe(params.gt_topic_name+params.target_odom_names[t], 10, &Node::GTTargetCallback, this));
    }*/

    update_timer = nh.createTimer(ros::Duration(params.max_update_time_rate), &Node::timer_callback, this);

    image_pub_ = it_.advertise("image_tracked", 1);
    image_debug_pub_ = it_.advertise("image_tracked_debug", 1);
    tracks_pub_ = nh.advertise<jpdaf_tracker_msgs::Tracks>("jpdaf_tracks", 1);

    track_init = true;
    
    for(int i=1; i<=params.nb_drones; i++)
    {
        lost_tracks.push_back(i);
    }

    R_cam_imu << -0.9994192917454484, -0.015152697184557497, 0.03052007626234578,
                0.003454872095575218, -0.9361296277702554, -0.351638143365486,
                0.03389901393594679, -0.3513285012331849, 0.9356383601987547;

    debug_track_counter = 0;

    ROS_INFO("Node initialized successfully");


    output_file_.open(params.root_ + params.output_file_name_ + ".txt", ios::out | ios::app);
    assert(output_file_.is_open());


}

void Node::timer_callback(const ros::TimerEvent& event)
{
        track(false);
}

void Node::detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes)
{
    bounding_boxes_msgs_buffer_.push_back(*bounding_boxes);
    track(true);
}

void Node::imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    image_buffer_.push_back(img_msg);
}

void Node::imuCallback(const sensor_msgs::Imu& imu_msg)
{
    imu_buffer_.push_back(imu_msg);
}

/*void Node::GTSourceCallback(const nav_msgs::OdometryConstPtr& msg)
{
}

void Node::GTTargetCallback(const nav_msgs::OdometryConstPtr& msg)
{
}*/



void Node::track(bool called_from_detection)
{
    update_timer.stop();

    ROS_INFO("---------------------------------------------");
    ROS_INFO("debug track counter: %d", debug_track_counter);
    if(!called_from_detection)
    {
        ROS_WARN("Tracking called from timer, no new detections");
    }

    darknet_ros_msgs::BoundingBoxes latest_detection;
    if(called_from_detection)
    {
        latest_detection = bounding_boxes_msgs_buffer_.back();
    }
    else
    {
        std::vector<darknet_ros_msgs::BoundingBox> emptyVector;
        latest_detection.bounding_boxes = emptyVector;
    }
    auto detections = get_detections(latest_detection);

    ROS_INFO("Detections:");
    for(int d=0; d<(int)detections.size(); d++)
    {
        cout << detections[d].getVect() << endl;
    }


    if(track_init)
    {
        ROS_WARN("Tracker not initialized");
        for (uint d=0; d<detections.size(); d++)
        {
            prev_unassoc_detections.push_back(detections[d]);
        }

        //create_tracks_test_input();//ttt       

        if(called_from_detection)
        {
            last_timestamp_synchronized = latest_detection.header.stamp.toSec();
            last_timestamp_from_rostime = ros::Time::now().toSec();
            last_track_from_detection = true;
            ROS_INFO("Tracker initialized");
            track_init = false;
        }
    }
    else
    {
        cout << "Currently tracked tracks indexes: " << endl; for(auto tr : tracks_){cout << tr.getId() << " ";} cout << endl;
        double time_step;
        if(called_from_detection && last_track_from_detection)
        {
            time_step = latest_detection.header.stamp.toSec() - last_timestamp_synchronized; // testing new timestamp method
        }
        else
        {
            time_step = ros::Time::now().toSec() - last_timestamp_from_rostime;
        }
        ROS_INFO("tracking called with time step %f, detection boxes nb: %d", time_step, (int)detections.size());

        if(time_step < 0)
        {
            ROS_FATAL("Negative time step! %f", time_step);// Should not happen anymore
            exit(0);    
        }
        if(time_step != 0.0)//Sometimes Darknet ros publishes consecutive detections (bounding boxes and detections) with same timestamp, this happens when no new image is available in the yolo's buffer of images to perform detection
        {

            auto omega = compute_angular_velocity((double)(last_timestamp_synchronized + time_step));
            
            //PREDICTION
            std::vector<Eigen::Vector2f> projected_predictions;
            for(uint t=0; t<tracks_.size(); t++)
            {
                tracks_[t].predict(time_step, omega);
                projected_predictions.push_back(tracks_[t].get_z());
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


            draw_tracks_publish_image(detections, (double)(last_timestamp_synchronized + time_step), projected_predictions);
            publishTracks((double)(last_timestamp_synchronized + time_step));

            writeToFile((int)detections.size(), omega);

            manage_new_old_tracks(detections, alphas_0, betas_0, omega, time_step); //ttt

            //Update of variables for next call
            if(called_from_detection)
            {
                last_timestamp_synchronized = latest_detection.header.stamp.toSec();
                last_track_from_detection = true;
            }
            else
            {
                last_timestamp_synchronized += ros::Time::now().toSec() - last_timestamp_from_rostime;
                last_track_from_detection = false;
            }
            last_timestamp_from_rostime = ros::Time::now().toSec();

        }

    }

    bounding_boxes_msgs_buffer_.clear();

    debug_track_counter++;

    update_timer.start();
}


Eigen::MatrixXf Node::association_matrix(const std::vector<Detection> detections)
{
    Eigen::MatrixXf q(detections.size(), tracks_.size()+1);
    q.setZero();

    for(uint i=0; i<detections.size(); i++) {q(i,0)=1;} // Setting first column to 1
    
    for(uint i=0; i<detections.size(); i++)
    {
        for (uint j=0; j<tracks_.size(); j++)
        {
            Eigen::Vector2f measure = detections[i].getVect();
            Eigen::Vector2f prediction = tracks_[j].get_z();
            if((measure-prediction).transpose() * tracks_[j].S().inverse() * (measure-prediction) <= pow(params.gamma, 2))
            {
                q(i, j+1)=1;
            }
        }
    }
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
            y_tilds.push_back(detections[measurement_index].getVect() - tracks_[track_index].get_z());
            //cout << "y_tild: " << endl << detections[measurement_index].getVect() - tracks_[track_index].get_z_predict() << endl;
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

void Node::manage_new_old_tracks(std::vector<Detection> detections, std::vector<double> alphas_0, std::vector<double> betas_0, Eigen::Vector3f omega, double time_step)
{
    //ROS_INFO("manage new old tracks: alphas_0 length=%d, betas_0 length=%d", (int)alphas_0.size(), (int)betas_0.size());
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

    auto new_tracks = create_new_tracks(detections, unassoc_detections_idx, omega, time_step);

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

std::vector<Track> Node::create_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections_idx, Eigen::Vector3f omega, double time_step)
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
            if( assignments[i] != -1 && costMat(i, assignments[i]) < params.assoc_cost)//TODO: should add offset due to motion into comparison 
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
                    Eigen::MatrixXf B;
                    B = Eigen::MatrixXf(2, 3);
                    B << 0, 0, 0,
                         0, 0, 0;

                    B(0,0) = ((unassoc_detections.at(j).x()-params.principal_point(0))*(unassoc_detections.at(j).y()-params.principal_point(1)))/params.focal_length;
                    B(0,1) = -(params.focal_length*params.alpha_cam + (unassoc_detections.at(j).x()-params.principal_point(0))*(unassoc_detections.at(j).x()-params.principal_point(0))/(params.focal_length*params.alpha_cam));
                    B(0,2) = params.alpha_cam*(unassoc_detections.at(j).y()-params.principal_point(1));
                    
                    B(1,0) = (params.focal_length + (unassoc_detections.at(j).y()-params.principal_point(1))*(unassoc_detections.at(j).y()-params.principal_point(1))/params.focal_length); 
                    B(1,1) = -((unassoc_detections.at(j).x()-params.principal_point(0))*(unassoc_detections.at(j).y()-params.principal_point(1)))/(params.alpha_cam*params.focal_length);
                    B(1,2) = -(unassoc_detections.at(j).x()-params.principal_point(0))/params.alpha_cam;

                    auto speed_offset = B*omega;

                    const float& vx = (unassoc_detections.at(j).x()-prev_unassoc_detections.at(i).x())/time_step - speed_offset(0); 
                    const float& vy = (unassoc_detections.at(j).y()-prev_unassoc_detections.at(i).y())/time_step - speed_offset(1);
                    Track tr(unassoc_detections.at(j).x(), unassoc_detections.at(j).y(), vx, vy, params);
                    new_tracks.push_back(tr);
                    ROS_INFO("created new track with position %f %f, speed %f %f", unassoc_detections.at(j).x(), unassoc_detections.at(j).y(), vx, vy);
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

Eigen::Vector3f Node::compute_angular_velocity(double detection_timestamp)
{
    ROS_INFO("Imu buffer length: %d", (int)imu_buffer_.size());
    //ROS_INFO("Detection timestamp: %f", detection_timestamp);

    if(!imu_buffer_ok(detection_timestamp))
    {
        Eigen::Vector3f omega(0, 0, 0);
        return omega;
    }

    int imu_buffer_index = 0;
    while(imu_buffer_index != (int)imu_buffer_.size()-1)
    {
        if((double)imu_buffer_[imu_buffer_index].header.stamp.toSec() >= detection_timestamp)
        {
            break;
        }
        imu_buffer_index++;
    }

    //ROS_INFO("Selected imu buffer timestamp: %f", imu_buffer_[imu_buffer_index].header.stamp.toSec());
    
    Eigen::Vector3f omega_imu;
    omega_imu << imu_buffer_[imu_buffer_index].angular_velocity.x, imu_buffer_[imu_buffer_index].angular_velocity.y, imu_buffer_[imu_buffer_index].angular_velocity.z;

    Eigen::Vector3f omega_cam;
    omega_cam = R_cam_imu*omega_imu;

    cout << "omega cam: " << endl << omega_cam << endl;

    std::vector<sensor_msgs::Imu> temp;
    for(int i=imu_buffer_index; i<(int)imu_buffer_.size(); i++)
    {
        temp.push_back(imu_buffer_[i]);
    }
    imu_buffer_.clear();
    imu_buffer_ = temp;

    return omega_cam;
}

bool Node::imu_buffer_ok(double detection_timestamp)
{
    if((int)imu_buffer_.size() > 1000)
    {
        ROS_FATAL("Imu buffer is too long, there is a problem somewhere");
        exit(1);
    }
    else if((int)imu_buffer_.size() == 0)
    {
        ROS_ERROR("Imu buffer length is 0. Assuming no orientation change");
        return false;
    }
    else if(detection_timestamp - imu_buffer_.back().header.stamp.toSec() > 0) 
    {
        ROS_ERROR("Imu buffer is running too late compaired to the detections");
//        exit(0);
        return false;
    }
    else if(detection_timestamp - imu_buffer_.front().header.stamp.toSec() < 0) 
    {
        ROS_WARN("Imu buffer doesn't contain elements prior to the detection. Assuming to angular velocity");
        return false;
    }
    else
    {
        return true;
    }
}

void Node::draw_tracks_publish_image(std::vector<Detection> detections, double detection_timestamp, std::vector<Eigen::Vector2f> projected_predictions)
{
    ROS_INFO("Image buffer length: %d", (int)image_buffer_.size());
    //ROS_INFO("Detection timestamp: %f", detection_timestamp);

    if(!image_buffer_ok(detection_timestamp))
    {
        ROS_INFO("No corresponding image in the buffer!");
        return;
    }

    int image_buffer_index = 0;
    while(image_buffer_index != (int)image_buffer_.size()-1)
    {
        if((double)image_buffer_[image_buffer_index]->header.stamp.toSec() >= detection_timestamp)
        {
            break;
        }
        image_buffer_index++;
    }

    //ROS_INFO("Selected image buffer timestamp: %f", image_buffer_[image_buffer_index]->header.stamp.toSec());

    auto sync_image_ptr = image_buffer_[image_buffer_index];

    cv_bridge::CvImageConstPtr im_ptr_ = cv_bridge::toCvShare(sync_image_ptr, "rgb8");
    cv::Mat im = im_ptr_->image;

    if(im.empty()) return;

    for(uint d=0; d<detections.size(); d++)
    {
        //cv::circle(im, detections[d](), 2, cv::Scalar(255, 20, 150), 2); 

        cv::Point2f det_cross_a(detections[d]().x-10, detections[d]().y-10);
        cv::Point2f det_cross_b(detections[d]().x+10, detections[d]().y-10);
        cv::Point2f det_cross_c(detections[d]().x-10, detections[d]().y+10);
        cv::Point2f det_cross_d(detections[d]().x+10, detections[d]().y+10);
        line(im, det_cross_a, det_cross_d, cv::Scalar(255, 20, 150), 2, 2 );
        line(im, det_cross_b, det_cross_c, cv::Scalar(255, 20, 150), 2, 2 );
    }


    for(uint p=0; p<projected_predictions.size(); p++)
    {
        if(tracks_[p].getId() != -1)
        {
            cv::Point2f pr_pos((int)(projected_predictions[p])(0), (int)(projected_predictions[p])(1));
            cv::circle(im, pr_pos, 2, cv::Scalar(0, 210, 255), 2); 
        }
        else
        {
            cv::Point2f pr_pos((int)(projected_predictions[p])(0), (int)(projected_predictions[p])(1));
            cv::circle(im, pr_pos, 1, cv::Scalar(180, 0, 255), 2); 
        }
    }

    for(uint t=0; t<tracks_.size(); t++)
    {
        if(tracks_[t].getId() != -1)
        {
            cv::Point2f tr_pos((int)(tracks_[t].get_z())(0), (int)(tracks_[t].get_z())(1));
            cv::Point2f id_pos(tr_pos.x, tr_pos.y+30);
            cv::circle(im, tr_pos, 5, cv::Scalar(0, 255, 0), 2); 
            putText(im, to_string(tracks_[t].getId()), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cvScalar(0, 255, 0), 1, CV_AA);

            //ellipse for 95% confidence
            cv::RotatedRect ellipse = tracks_[t].get_error_ellipse(2.4477);
            cv::ellipse(im, ellipse, cv::Scalar(150, 255, 150), 1);
        }
        else
        {
            cv::Point2f tr_pos((int)(tracks_[t].get_z())(0), (int)(tracks_[t].get_z())(1));
            cv::Point2f id_pos(tr_pos.x, tr_pos.y+30);
            cv::circle(im, tr_pos, 5, cv::Scalar(255, 150, 0), 2);
            putText(im, "-", id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cvScalar(255, 150, 0), 1, CV_AA);

            //ellipse for 95% confidence
            cv::RotatedRect ellipse = tracks_[t].get_error_ellipse(2.4477);
            cv::ellipse(im, ellipse, cv::Scalar(255, 250, 150), 1);
        }
    }

    cv_bridge::CvImage processed_image_bridge;
    processed_image_bridge.header.stamp = sync_image_ptr->header.stamp;
    processed_image_bridge.image = im;
    processed_image_bridge.encoding = sensor_msgs::image_encodings::RGB8;
    sensor_msgs::ImagePtr im_msg = processed_image_bridge.toImageMsg();
    image_pub_.publish(im_msg);


    cv::Point2f counter_pos(10, 30);
    putText(im, to_string(debug_track_counter), counter_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 0, 0), 1, CV_AA);

    cv_bridge::CvImage processed_image_debug_bridge;
    processed_image_debug_bridge.header.stamp = sync_image_ptr->header.stamp;
    processed_image_debug_bridge.image = im;
    processed_image_debug_bridge.encoding = sensor_msgs::image_encodings::RGB8;
    sensor_msgs::ImagePtr im_debug_msg = processed_image_debug_bridge.toImageMsg();
    image_debug_pub_.publish(im_debug_msg);


    std::vector<sensor_msgs::ImageConstPtr> temp;
    for(int i=image_buffer_index; i<(int)image_buffer_.size(); i++)
    {
        temp.push_back(image_buffer_[i]);
    }
    image_buffer_.clear();
    image_buffer_ = temp;

    return;
}


bool Node::image_buffer_ok(double detection_timestamp)
{
    if((int)image_buffer_.size() > 1000)
    {
        ROS_FATAL("Image buffer is too long, there is a problem somewhere");
        exit(1);
    }
    else if((int)image_buffer_.size() == 0)
    {
        ROS_ERROR("Image buffer length is 0.");
        return false;
    }
    else if(detection_timestamp - image_buffer_.back()->header.stamp.toSec() > 0) 
    {
        ROS_FATAL("Image buffer is running too late compaired to the detections");
        //exit(0);
        return false;
    }
    else if(detection_timestamp - image_buffer_.front()->header.stamp.toSec() < 0) 
    {
        ROS_WARN("Image buffer doesn't contain elements prior to the detections");
        return false;
    }
    else
    {
        return true;
    }
}

void Node::publishTracks(double detection_timestamp)
{

    jpdaf_tracker_msgs::Tracks trs_msg;
    for(uint t=0; t<tracks_.size(); t++)
    {
        if(tracks_[t].getId() != -1)
        {
            jpdaf_tracker_msgs::Track tr_msg;
            tr_msg.id = tracks_[t].getId();
            tr_msg.x = (int)(tracks_[t].get_z())(0);
            tr_msg.y = (int)(tracks_[t].get_z())(1);
            
            trs_msg.tracks.push_back(tr_msg);
        }
    }
    ros::Time timestamp(detection_timestamp);
    trs_msg.header.stamp = timestamp;
    
    tracks_pub_.publish(trs_msg);
}

void Node::writeToFile(int nb_detections, Eigen::Vector3f omega_cam)
{
    output_file_ << nb_detections << " " << (int)tracks_.size() << " " << omega_cam.x() << " " << omega_cam.y() << " " << omega_cam.z() << endl;
    return;
}


std::vector<Detection> Node::get_detections(const darknet_ros_msgs::BoundingBoxes latest_detection)
{
    std::vector<Detection> norm_det;
    for(uint i=0; i<latest_detection.bounding_boxes.size(); i++)
    {
        Detection one_det(float(latest_detection.bounding_boxes[i].xmin+latest_detection.bounding_boxes[i].xmax)/2, 
                          float(latest_detection.bounding_boxes[i].ymin+latest_detection.bounding_boxes[i].ymax)/2, 
                          latest_detection.bounding_boxes[i].xmax-latest_detection.bounding_boxes[i].xmin, 
                          latest_detection.bounding_boxes[i].ymax-latest_detection.bounding_boxes[i].ymin);
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

void Node::create_tracks_test_input()
{
    Track tr1(0, 0, 0, 0, params);
    tracks_.push_back(tr1);
    Track tr2(240, 0, 0, 0, params);
    tracks_.push_back(tr2);
    Track tr3(480, 0, 0, 0, params);
    tracks_.push_back(tr3);
    Track tr4(720, 0, 0, 0, params);
    tracks_.push_back(tr4);
    Track tr5(960, 0, 0, 0, params);
    tracks_.push_back(tr5);
    Track tr6(0, 135, 0, 0, params);
    tracks_.push_back(tr6);
    Track tr7(240, 135, 0, 0, params);
    tracks_.push_back(tr7);
    Track tr8(480, 135, 0, 0, params);
    tracks_.push_back(tr8);
    Track tr9(720, 135, 0, 0, params);
    tracks_.push_back(tr9);
    Track tr10(960, 135, 0, 0, params);
    tracks_.push_back(tr10);
    Track tr11(0, 270, 0, 0, params);
    tracks_.push_back(tr11);
    Track tr12(240, 270, 0, 0, params);
    tracks_.push_back(tr12);
    Track tr13(480, 270, 0, 0, params);
    tracks_.push_back(tr13);
    Track tr14(720, 270, 0, 0, params);
    tracks_.push_back(tr14);
    Track tr15(960, 270, 0, 0, params);
    tracks_.push_back(tr15);
    Track tr16(0, 405, 0, 0, params);
    tracks_.push_back(tr16);
    Track tr17(240, 405, 0, 0, params);
    tracks_.push_back(tr17);
    Track tr18(480, 405, 0, 0, params);
    tracks_.push_back(tr18);
    Track tr19(720, 405, 0, 0, params);
    tracks_.push_back(tr19);
    Track tr20(960, 405, 0, 0, params);
    tracks_.push_back(tr20);
    Track tr21(0, 540, 0, 0, params);
    tracks_.push_back(tr21);
    Track tr22(240, 540, 0, 0, params);
    tracks_.push_back(tr22);
    Track tr23(480, 540, 0, 0, params);
    tracks_.push_back(tr23);
    Track tr24(720, 540, 0, 0, params);
    tracks_.push_back(tr24);
    Track tr25(960, 540, 0, 0, params);
    tracks_.push_back(tr25);
}



}
