#include "mbed.h"
#include "MPU6050.h"
#include "math.h"
#include "Eigen/Dense.h"

#define M_PI 3.14159265358979323846
#define N 3

//debug matrix
#define PRINT_MATRIX(MATRIX)  \
 std::printf(#MATRIX "_matrix\r\n"); \
 for (int i = 0; i < MATRIX.rows(); ++i) \
 { \
   for (int j = 0; j < MATRIX.cols(); ++j) \
   { \
     std::printf("%.3f\t", MATRIX(i, j)); \
   } \
   std::printf("\r\n");             \
 } \

using namespace Eigen;
Serial pc(USBTX, USBRX);
MPU6050 mpu(p28,p27);   //sda,scl
Timer t1;
//LocalFileSystem local("local");

int main()
{
  //FILE *fp = fopen("/local/result.csv", "w");

  pc.printf("\r\n---------------------start program---------------------");
  pc.printf("\r\n");

  t1.start();
  float gyro[N]={};
  float accel[N]={};
  mpu.getGyro(gyro);
  mpu.getAccelero(accel);

  float g = 9.80665;
  float sigma_v[4]={0.01, 0.01, 0.01, 0.01};
  float sigma_w[3]={1.0, 1.0, 10.0};//{0.0013684641, 0.0012666226, 0.003095776};

  float Deg[3] = {};
  float W_x[2]={-0.0367934,gyro[0]};
  float W_y[2]={0.08030829,gyro[1]};
  float W_z[2]={0.00066347,gyro[2]};
  float a_x;
  float a_y;
  float a_z;

  MatrixXf preX(4,1);
  MatrixXf X(4,1);
  MatrixXf y(3,1);
  MatrixXf f(4,4);
  MatrixXf h(3,1);
  MatrixXf A(4,4);
  MatrixXf c(3,4);
  MatrixXf E = MatrixXf::Identity(4,4);
  MatrixXf SigmaV(4,4); //system dispersion
  MatrixXf SigmaW(3,3); //observe dispersion

  MatrixXf C(3,3);
  
  SigmaV << sigma_v[0],          0,          0,          0,
                     0, sigma_v[1],          0,          0,
                     0,          0, sigma_v[2],          0, 
                     0,          0,          0, sigma_v[3];
  
  SigmaW << sigma_w[0],          0,          0,
                     0, sigma_w[1],          0,
                     0,          0, sigma_w[2];
  
  MatrixXf P = 0.01*E;
  //P => P_1
  MatrixXf P_1 = P;

  X << 0,
       0,
       0,
       1.0;
  MatrixXf X_1 = X;
  
  while(true)
    {
    /*-----------update-----------*/
    //timer
    double T_s = t1.read();
    t1.reset();

    //mpu
    mpu.getGyro(gyro);
    W_x[1] = gyro[0];
    W_y[1] = gyro[1];
    W_z[1] = gyro[2];
    mpu.getAccelero(accel);
    a_x = accel[0];
    a_y = accel[1];
    a_z = accel[2];
    
    /*-----------calc-----------*/
    /*step1*/
    A << 1,                T_s*W_z[0]/2,     -1*T_s*W_y[0]/2,  T_s*W_x[0]/2,
         -1*T_s*W_z[0]/2,  1,                T_s*W_x[0]/2,     T_s*W_y[0]/2,
         T_s*W_y[0]/2,     -1*T_s*W_x[0]/2,  1,                T_s*W_z[0]/2,                   
         -1*T_s*W_x[0]/2,  -1*T_s*W_y[0]/2,  -1*T_s*W_z[0]/2,  1;
    
    f = A*X_1;
    
    preX = f;
    
    h << 2*g*(preX(2,0)*preX(0,0)-preX(1,0)*preX(3,0)),
         2*g*(preX(1,0)*preX(2,0)+preX(0,0)*preX(3,0)),
         g*(pow(preX(2,0),2) - pow(preX(0,0),2) - pow(preX(1,0),2) - pow(preX(3,0),2));
    
    c << 2*g*preX(2,0),   -2*g*preX(3,0),  2*g*preX(0,0),  -2*g*preX(1,0),
         2*g*preX(3,0),   2*g*preX(2,0),   2*g*preX(1,0),  2*g*preX(0,0),
         -2*g*preX(0,0),  -2*g*preX(1,0),  2*g*preX(2,0),  2*g*preX(3,0);
    
    MatrixXf preP = A*P_1*A.transpose()+SigmaV;
    
    /*step2*/
    //kalman gain g
    MatrixXf Q = preP*c.transpose();
    MatrixXf q = c*Q+SigmaW;
    MatrixXf g = Q*q.inverse();
    
    y << a_x,
         a_y,
         a_z;
    
    MatrixXf X = preX + g*(y-h);
    
    //P
    MatrixXf P = (E - g*c)*preP;
    P_1 = P;
    
    //DCM to Quaternion
    C << (pow(X(0,0),2)-pow(X(1,0),2)-pow(X(2,0),2)+pow(X(3,0),2)),  2*(X(0,0)*X(1,0)+X(2,0)*X(3,0)),                            2*(X(2,0)*X(0,0)-X(1,0)*X(3,0)),
         2*(X(0,0)*X(1,0)-X(2,0)*X(3,0)),                            (pow(X(1,0),2)-pow(X(2,0),2)-pow(X(0,0),2)+pow(X(3,0),2)),  2*(X(1,0)*X(2,0)+X(0,0)*X(3,0)),
         2*(X(2,0)*X(0,0)+X(1,0)*X(3,0)),                            2*(X(1,0)*X(2,0)-X(0,0)*X(3,0)),                            (pow(X(2,0),2)-pow(X(0,0),2)-pow(X(1,0),2)+pow(X(3,0),2));
    
    //roll, pitch, yaw
    Deg[1] = asin(-1*C(0,2));                                  //pitch
    Deg[0] = atan2(C(0,1)/cos(Deg[1]), C(0,0)/cos(Deg[1]));    //yaw
    Deg[2] = atan2(C(1,2)/cos(Deg[1]), C(2,2)/cos(Deg[1]));    //roll
    
    /*-----------draw-----------*/    
    //fprintf(fp, "kalman \t %f,%f,%f \t observe \t %f,%f\r\n", Deg[0], Deg[1], Deg[2], -1*atan2(a_x,sqrt(pow(a_y,2)+pow(a_z,2))), atan2(a_y,a_z));
    pc.printf("kalman \t %f,%f,%f \t observe \t %f,%f\r\n", Deg[0], Deg[1], Deg[2], -1*atan2(a_x,sqrt(pow(a_y,2)+pow(a_z,2))), atan2(a_y,a_z));
    
    //pass param
    W_x[0]=W_x[1];
    W_y[0]=W_y[1];
    W_z[0]=W_z[1];
    }
}
