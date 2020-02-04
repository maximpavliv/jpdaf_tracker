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
    life_time = 0;
}


void Track::predict(float dt)
{
    KF->predict(dt);  
}


void Track::gainUpdate()
{
    KF->gainUpdate();
}

void Track::update(const std::vector<Detection> detections, std::vector<double> beta, double beta_0)
{
    float sum_betas = 0; //TODO change here, because if tracks are near to other detection associated, they wont be lost. Do check with beta_0 > some_threshold!! Also need to do a similar change in manage_new_tracks!!
    for(auto beta_i : beta)
    {
        sum_betas += beta_i;
    }
    if(sum_betas == 0)
        noDetections++;
    else
        noDetections = 0;    

    life_time++;

    KF->update(detections, beta, beta_0);
}
