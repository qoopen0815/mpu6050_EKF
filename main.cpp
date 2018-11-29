#include "mbed.h"
#include "MPU6050.h"
#include "math.h"
#include "Matrix.h"
#include "MatrixMath.h"

#define M_PI 3.14159265358979323846
#define N 3

Serial pc(USBTX, USBRX);
MPU6050 mpu(p28,p27);   //sda,scl
Timer t1;

int main()
{
  pc.printf("start program");
  pc.printf("\r\n");
  t1.start();
  float gyro[N]={};
  float accel[N]={};
  float sigma_v = 0.5, sigma_w = 0.5;
  float thetahat_r[2]={};
  float thetahat_p[2]={};
  float prethetahat_r[2]={};
  float prethetahat_p[2]={};
  float W_x[2]={};
  float W_y[2]={};
  float W_z[2]={};
  float What_x[2]={};
  float What_y[2]={};
  float What_z[2]={};
  float preWhat_x[2]={};
  float preWhat_y[2]={};
  float preWhat_z[2]={};
  float a_x[2]={};
  float a_y[2]={};
  float a_z[2]={};
  float preahat_x[2]={};
  float preahat_y[2]={};
  float preahat_z[2]={};
  float y_val[8][1]={};
  float f_val[8][1]={};
  float h_val[8][1]={};
  float A_val[8][8]={};
  float C_val[8][8]={};

  Matrix y(8,1);
  Matrix f(8,1);
  Matrix h(8,1);
  Matrix A(8,8);
  Matrix B(8,8);
  Matrix C(8,8);
  Matrix E(8,8);  
    
  // Fill Matrix with data.
  E << 1            << 0            << 0            << 0            << 0            << 0            << 0            << 0
    << 0            << 1            << 0            << 0            << 0            << 0            << 0            << 0
    << 0            << 0            << 1            << 0            << 0            << 0            << 0            << 0
    << 0            << 0            << 0            << 1            << 0            << 0            << 0            << 0         
    << 0            << 0            << 0            << 0            << 1            << 0            << 0            << 0         
    << 0            << 0            << 0            << 0            << 0            << 1            << 0            << 0         
    << 0            << 0            << 0            << 0            << 0            << 0            << 1            << 0         
    << 0            << 0            << 0            << 0            << 0            << 0            << 0            << 1;
    
  Matrix P(E);

  while(true) {
    /*-----------update-----------*/
    //timer
    double T_s = t1.read();
    t1.reset();
    //pass param
    thetahat_r[0]=thetahat_r[1];
    thetahat_p[0]=thetahat_p[1];
    prethetahat_r[0]=prethetahat_r[1];
    prethetahat_p[0]=prethetahat_p[1];
    W_x[0]=W_x[1];
    W_y[0]=W_y[1];
    W_z[0]=W_z[1];
    What_x[0]=What_x[1];
    What_y[0]=What_y[1];
    What_z[0]=What_z[1];
    preWhat_x[0]=preWhat_x[1];
    preWhat_y[0]=preWhat_y[1];
    preWhat_z[0]=preWhat_z[1];
    a_x[0]=a_x[1];
    a_y[0]=a_y[1];
    a_z[0]=a_z[1];
    preahat_x[0]=preahat_x[1];
    preahat_y[0]=preahat_y[1];
    preahat_z[0]=preahat_z[1];
    //mpu
    mpu.getGyro(gyro);
    W_x[1] = gyro[0];
    W_y[1] = gyro[1];
    W_z[1] = gyro[2];
    mpu.getAccelero(accel);
    a_x[1] = accel[0];
    a_y[1] = accel[1];
    a_z[1] = accel[2];
    //y_val
    y_val[0][0] = -1*atan2(a_x[1],sqrt(pow(a_y[1],2)+pow(a_z[1],2)));
    y_val[1][0] = atan2(a_y[1],a_z[1]);
    y_val[2][0] = W_x[1];
    y_val[3][0] = W_y[1];
    y_val[4][0] = W_z[1];
    y_val[5][0] = a_x[1];
    y_val[6][0] = a_y[1];
    y_val[7][0] = a_z[1];
    //f_val
    f_val[0][0] = thetahat_p[0]+(What_y[0]*cos(thetahat_r[0])-What_z[0]*sin(thetahat_r[0]))*T_s;
    f_val[1][0] = thetahat_r[0]+(What_x[0]+(What_y[0]*sin(thetahat_r[0])+What_z[0]*cos(thetahat_r[0]))*tan(thetahat_p[0]))*T_s;
    f_val[2][0] = 0;
    f_val[3][0] = 0;
    f_val[4][0] = 0;
    f_val[5][0] = 0;
    f_val[6][0] = 0;
    f_val[7][0] = 0;
    //h_val
    h_val[0][0] = -1*atan2(preahat_x[1],sqrt(pow(preahat_y[1],2)+pow(preahat_z[1],2)));
    h_val[1][0] = atan2(preahat_y[1],preahat_z[1]);
    h_val[2][0] = preWhat_x[1];
    h_val[3][0] = preWhat_y[1];
    h_val[4][0] = preWhat_z[1];
    h_val[5][0] = preahat_x[1];
    h_val[6][0] = preahat_y[1];
    h_val[7][0] = preahat_z[1];
    //A_val
    A_val[0][0] = 1;
    A_val[0][1] = (-preWhat_y[0]*sin(prethetahat_r[0])-preWhat_z[0]*cos(prethetahat_r[0]))*T_s;
    A_val[0][2] = 0;
    A_val[0][3] = T_s*cos(prethetahat_r[0]);
    A_val[0][4] = -1*T_s*sin(prethetahat_r[0]);
    A_val[0][5] = 0;
    A_val[0][6] = 0;
    A_val[0][7] = 0;
    A_val[1][0] = ((preWhat_y[0]*sin(prethetahat_r[0]))/(pow(cos(prethetahat_p[0]),2))+(preWhat_z[0]*cos(prethetahat_r[0]))/(pow(cos(prethetahat_p[0]),2)))*T_s;
    A_val[1][1] = 1+(preWhat_y[0]*cos(prethetahat_r[0])-preWhat_z[0]*sin(prethetahat_r[0]))*T_s*tan(prethetahat_p[0]);
    A_val[1][2] = T_s;
    A_val[1][3] = T_s*sin(prethetahat_r[0])*tan(prethetahat_p[0]);
    A_val[1][4] = T_s*cos(prethetahat_r[0])*tan(prethetahat_p[0]);
    A_val[1][5] = 0;
    A_val[1][6] = 0;
    A_val[1][7] = 0;
    //C_val
    C_val[0][0] = 0;
    C_val[0][1] = 0;
    C_val[0][2] = 0;
    C_val[0][3] = 0;
    C_val[0][4] = 0;
    C_val[0][5] = -1*sqrt(pow(preahat_y[1],2))+pow(preahat_z[1],2)/(pow(preahat_x[1],2)+pow(preahat_y[1],2)+pow(preahat_z[1],2));
    C_val[0][6] = preahat_x[1]*preahat_y[1]/(sqrt(pow(preahat_y[1],2)+pow(preahat_z[1],2))*(pow(preahat_x[1],2)+pow(preahat_y[1],2)+pow(preahat_z[1],2)));
    C_val[0][7] = preahat_x[1]*preahat_z[1]/(sqrt(pow(preahat_y[1],2)+pow(preahat_z[1],2))*(pow(preahat_x[1],2)+pow(preahat_y[1],2)+pow(preahat_z[1],2)));
    C_val[1][0] = 0;
    C_val[1][1] = 0;
    C_val[1][2] = 0;
    C_val[1][3] = 0;
    C_val[1][4] = 0;
    C_val[1][5] = 0;
    C_val[1][6] = preahat_z[1]/(pow(preahat_y[1],2)+pow(preahat_z[1],2));
    C_val[1][7] = -1*preahat_y[1]/(pow(preahat_y[1],2)+pow(preahat_z[1],2));
    //fill y_matrix
    y << y_val[0][0]
      << y_val[1][0]
      << y_val[2][0]         
      << y_val[3][0]         
      << y_val[4][0]         
      << y_val[5][0]           
      << y_val[6][0]           
      << y_val[7][0];
    //fill f_matrix
    f << f_val[0][0]
      << f_val[1][0]
      << f_val[2][0]         
      << f_val[3][0]         
      << f_val[4][0]         
      << f_val[5][0]           
      << f_val[6][0]           
      << f_val[7][0];
    //fill h_matrix
    h << h_val[0][0]
      << h_val[1][0]
      << h_val[2][0]         
      << h_val[3][0]         
      << h_val[4][0]         
      << h_val[5][0]           
      << h_val[6][0]           
      << h_val[7][0];
    //fill A_matrix
    A << A_val[0][0]  << A_val[0][1]  << A_val[0][2]  << A_val[0][3]  << A_val[0][4]  << A_val[0][5]  << A_val[0][6]  << A_val[0][7]
      << A_val[1][0]  << A_val[1][1]  << A_val[1][2]  << A_val[1][3]  << A_val[1][4]  << A_val[1][5]  << A_val[1][6]  << A_val[1][7]
      << 0            << 0            << 1            << 0            << 0            << 0            << 0            << 0         
      << 0            << 0            << 0            << 1            << 0            << 0            << 0            << 0         
      << 0            << 0            << 0            << 0            << 1            << 0            << 0            << 0         
      << 0            << 0            << 0            << 0            << 0            << 1            << 0            << 0         
      << 0            << 0            << 0            << 0            << 0            << 0            << 1            << 0         
      << 0            << 0            << 0            << 0            << 0            << 0            << 0            << 1;
    //fill B_matrix
    B << 1            << 0            << 0            << 0            << 0            << 0            << 0            << 0
      << 0            << 1            << 0            << 0            << 0            << 0            << 0            << 0
      << 0            << 0            << 1            << 0            << 0            << 0            << 0            << 0
      << 0            << 0            << 0            << 1            << 0            << 0            << 0            << 0         
      << 0            << 0            << 0            << 0            << 1            << 0            << 0            << 0         
      << 0            << 0            << 0            << 0            << 0            << 1            << 0            << 0         
      << 0            << 0            << 0            << 0            << 0            << 0            << 1            << 0         
      << 0            << 0            << 0            << 0            << 0            << 0            << 0            << 1;
    //fill C_matrix
    C << C_val[0][0]  << C_val[0][1]  << C_val[0][2]  << C_val[0][3]  << C_val[0][4]  << C_val[0][5]  << C_val[0][6]  << C_val[0][7]
      << C_val[1][0]  << C_val[1][1]  << C_val[1][2]  << C_val[1][3]  << C_val[1][4]  << C_val[1][5]  << C_val[1][6]  << C_val[1][7]
      << 0            << 0            << 1            << 0            << 0            << 0            << 0            << 0         
      << 0            << 0            << 0            << 1            << 0            << 0            << 0            << 0         
      << 0            << 0            << 0            << 0            << 1            << 0            << 0            << 0         
      << 0            << 0            << 0            << 0            << 0            << 1            << 0            << 0         
      << 0            << 0            << 0            << 0            << 0            << 0            << 1            << 0         
      << 0            << 0            << 0            << 0            << 0            << 0            << 0            << 1;
	
    Matrix P_1(P);
	
    /*-----------calc-----------*/
    /*step1*/
    //prexhat
    Matrix prexhat(f);
    //preP
    Matrix preP = A*P_1*MatrixMath::Transpose(A)+pow(sigma_v,2)*B;

    /*step2*/
    //kalman gain g
    Matrix g = preP*C*MatrixMath::det(MatrixMath::Transpose(C)*preP*C+pow(sigma_w,2));
    Matrix xhat = prexhat + g*(y-h);
    Matrix P = (E - g*MatrixMath::Transpose(C))*preP;

    /*-----------draw-----------*/
    //      pc.printf("EKF: Roll:%.4f \t Pitch:%.4f",Roll,Pitch);
    float roll = xhat.getNumber(0,0);
    float pitch = xhat.getNumber(1,0);
    pc.printf("roll:%.3f\tpitch:%.3f",roll ,pitch);
    pc.printf("\r\n");
  }
}
