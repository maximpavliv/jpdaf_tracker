#include "track.h"

using namespace jpdaf;

Track::Track(const float& x, const float& y, const float& vx, const float& vy, TrackerParam params)
{
    KF = std::shared_ptr<Kalman>(new Kalman(x, y, vx, vy, params));
    life_time = 0;
    noDetections = 0;
    maxMissedRate = params.max_missed_rate;
    minAcceptanceRate = params.min_acceptance_rate;
    id = -1;
}


void Track::predict(float dt)
{
    ROS_INFO("Predicting track %d", id);
    KF->predict(dt);  
}


void Track::gainUpdate()
{
    ROS_INFO("Gaining track %d", id);
    KF->gainUpdate();
}

void Track::update(const std::vector<Detection> detections, std::vector<double> beta, double beta_0)
{
    ROS_INFO("Updating track %d", id);
    KF->update(detections, beta, beta_0);
}
