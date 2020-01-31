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

void Track::update(const std::vector<Detection> detections, std::vector<float> beta, float beta_0)
{
    //noDetections = 0;
    //TODO Add no Detections handling!!
    life_time++;
    KF->update(detections, beta, beta_0);
}
