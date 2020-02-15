#include <jpdaf_tracker/kalman.h>

using namespace std;

namespace jpdaf{

Kalman::Kalman(const float& x, const float& y, const float& vx, const float& vy, TrackerParam params)
{

  //CONTROL INPUT model Matrix
  B = Eigen::MatrixXf(4, 2);
  B << 1, 0,
       0, 0,
       0, 1,
       0, 0;

  //STATE OBSERVATION MATRIX
  C = Eigen::MatrixXf(2, 4);
  C << 1, 0, 0, 0,
       0, 0, 1, 0;
  
  //INITIAL COVARIANCE MATRIX
  //GAIN     
  K = Eigen::MatrixXf(4, 2);
  
  //measurement noise covariance matrix
  R = params.R;

  T = params.T;

  x_update << x, vx, y, vy;
  P_update = params.P_0;

  //cout << "x_update" << endl << x_update << endl;
  //cout << "P_update" << endl << P_update << endl;

  
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

//Need to write input here
  

  
  x_predict = A*x_update;
  //cout << "P_update: " << endl << P_update << endl;
  P_predict = A * P_update * A.transpose() + Q;
  //cout << "P_predict: " << endl << P_predict << endl;
  if(P_predict.determinant() < 0)
  {
    ROS_ERROR("Predicted covariance determinant is negative! %f", P_predict.determinant());
  }

  z_predict = C * x_predict;

  //Error Measurement Covariance Matrix
  S = C * P_predict * C.transpose() + R;
  cout << "S: " << endl << S << endl;    


  return;
}

void Kalman::gainUpdate()
{
  K = P_predict * C.transpose() * S.inverse();
}


void Kalman::update(const std::vector<Detection> detections, const std::vector<double> beta, double beta_0)
{
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

  //cout << "x_update" << endl << x_update << endl;

  Eigen::Matrix4f P_c;
  P_c = P_predict - K * S * K.transpose(); //Changed here, there is an error in the PhD thesis! It should be - instead of +

  Eigen::Matrix4f P_tild;
  Eigen::Matrix2f temp_sum;
  temp_sum << 0, 0, 0, 0;

  for(uint i=0; i<detections.size(); i++)
  {
      temp_sum += beta[i]*nus[i]*nus[i].transpose();
  }
  temp_sum -= nu*nu.transpose();
  P_tild = K * temp_sum * K.transpose();

                
  P_update = beta_0*P_predict + (1-beta_0)*P_c + P_tild;
  if(P_update.determinant() < 0)
  {
    ROS_ERROR("Update covariance determinant is negative! %f", P_update.determinant());
  }
  //cout << "P_update" << endl << P_update << endl;

  z_update = C * x_update;

}


}

