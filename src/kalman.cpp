#include <jpdaf_tracker/kalman.h>

using namespace std;

namespace jpdaf{

Kalman::Kalman(const float& x, const float& y, const float& vx, const float& vy, TrackerParam params)
{
  //STATE OBSERVATION MATRIX
  C = Eigen::MatrixXf(2, 4);
  C << 1, 0, 0, 0,
       0, 0, 1, 0;
  
  //INITIAL COVARIANCE MATRIX
  //P_init = params.P_0;
  //GAIN     
  K = Eigen::MatrixXf(4, 2);
  
  //measurement noise covariance matrix
  R = params.R;

  T = params.T;

  x_update << x, vx, y, vy;
  P_update = params.P_0;
  
  //init_prediction = cv::Point2f(x, y); //added by Max
  //init_speed = cv::Point2f(vx, vy); //added by Max
  //first_predict = true;
  //first_update = true;
}


void Kalman::predict(const float dt) //added by Max
{
  Eigen::Matrix4f A;
  A << 1, dt, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, dt,
       0, 0, 0, 1;
  Eigen::MatrixXf G;
  G = Eigen::MatrixXf(4, 2);
  G << std::pow(dt, 2) / 2, 0,
        	dt, 0,
        	0, std::pow(dt, 2) / 2,
        	0, dt;
  Eigen::Matrix4f Q;
  Q = G * T * G.transpose();
  
  /*if(first_predict)
  {
    x_predict << init_prediction.x, init_speed.x, init_prediction.y, init_speed.y; //added by Max
    P_predict = P_init;
    first_predict = false;
  }
  else
  {*/
    x_predict = A*x_update;
    P_predict = A * P_update * A.transpose() + Q;

//    cout << "P_predict: " << endl << P_predict << endl;

  //}

  z_predict = C * x_predict;

  //Error Measurement Covariance Matrix
//  cout << "P_update: " << endl << P_update << endl;
  S = C * P_predict * C.transpose() + R;

//  cout << "S: " << endl << S << endl;
    
  return; // added by Max
}

void Kalman::gainUpdate()
{
  K = P_predict * C.transpose() * S.inverse();
}


void Kalman::update(const std::vector<Detection> detections, const std::vector<double> beta, double beta_0)
{
    /*if(first_update)
    {
        first_update = false;
        return;
    }*/

    std::vector<Eigen::Vector2f> nus;
    for(uint i=0; i<detections.size(); i++)
    {
        nus.push_back(detections[i].getVect() - z_predict);
    }

    Eigen::Vector2f nu;
    nu << 0, 0;
    for(uint i=0; i<detections.size(); i++)
    {
        nu += beta[i] * nus[i];
    }
    
    x_update = x_predict + K * nu;

    Eigen::Matrix4f P_c;
    P_c = P_predict - K * S * K.transpose(); //Changed here, there is an error in the PhD thesis! It should be - instead of +
/*    cout << "computation of P_c:" << endl;
    cout << "P_predict" << endl << P_predict << endl;
    cout << "S" << endl << S << endl;
    cout << "K" << endl << K << endl;*/


    Eigen::Matrix4f P_tild;
    Eigen::Matrix2f temp_sum;
    temp_sum << 0, 0, 0, 0;
    for(uint i=0; i<detections.size(); i++)
    {
        temp_sum += beta[i]*nus[i]*nus[i].transpose() - nu*nu.transpose();
    }
    P_tild = K * temp_sum * K.transpose();
                
/*    cout << "P_predict: " << endl << P_predict << endl;
    cout << "P_c: " << endl << P_c << endl;
    cout << "P_tild: " << endl << P_tild << endl;*/

    P_update = beta_0*P_predict + (1-beta_0)*P_c + P_tild;

//    cout << "P_update: " << endl << P_update << endl;


}


}
































