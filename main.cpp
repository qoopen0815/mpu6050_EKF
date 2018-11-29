#include "mbed.h"
#include "MPU6050.h"
#include "math.h"
#include "Eigen/Dense.h"

#define M_PI 3.14159265358979323846
#define N 3

using namespace Eigen;
Serial pc(USBTX, USBRX);
MPU6050 mpu(p28,p27);   //sda,scl
Timer t1;

int main()
{
  pc.printf("start program");
  pc.printf("\r\n");

  pc.printf("prepare param");
  t1.start();
  float gyro[N]={};
  float accel[N]={};
  float sigma_v = 0.5, sigma_w = 0.5;
  float roll, pitch;
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
  pc.printf("\t done\r\n");

  pc.printf("prepare matrix");
  MatrixXf y(8,1);
  MatrixXf f(8,1);
  MatrixXf h(8,1);
  MatrixXf A(8,8);
  MatrixXf B(8,8);
  MatrixXf C(8,8);
  MatrixXf E(8,8);
  pc.printf("\t done\r\n");
    
  // Fill Matrix with data.
  E << 1,              0,              0,              0,              0,              0,              0,              0,
       0,              1,              0,              0,              0,              0,              0,              0,
       0,              0,              1,              0,              0,              0,              0,              0,
       0,              0,              0,              1,              0,              0,              0,              0,        
       0,              0,              0,              0,              1,              0,              0,              0,       
       0,              0,              0,              0,              0,              1,              0,              0,      
       0,              0,              0,              0,              0,              0,              1,              0,     
       0,              0,              0,              0,              0,              0,              0,              1;
    
  MatrixXf P(E);

  while(true)
    {
    /*-----------update-----------*/
    //timer
    pc.printf("start timer");
    double T_s = t1.read();
    t1.reset();
    pc.printf("\t done\r\n");

    //pass param
    pc.printf("pass param");
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
    pc.printf("\t done\r\n");

    //mpu
    pc.printf("give mpu");
    mpu.getGyro(gyro);
    W_x[1] = gyro[0];
    W_y[1] = gyro[1];
    W_z[1] = gyro[2];
    mpu.getAccelero(accel);
    a_x[1] = accel[0];
    a_y[1] = accel[1];
    a_z[1] = accel[2];
    pc.printf("\t done\r\n");

    pc.printf("give matrix_val");
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
    pc.printf("\t done\r\n");

    pc.printf("fill matrix");
    //fill y_matrix
    y << y_val[0][0],
         y_val[1][0],
         y_val[2][0],        
         y_val[3][0],       
         y_val[4][0],         
         y_val[5][0],           
         y_val[6][0],           
         y_val[7][0];
    //fill f_matrix
    f << f_val[0][0],
         f_val[1][0],
         f_val[2][0],         
         f_val[3][0],         
         f_val[4][0],         
         f_val[5][0],           
         f_val[6][0],           
         f_val[7][0];
    //fill h_matrix
    h << h_val[0][0],
         h_val[1][0],
         h_val[2][0],         
         h_val[3][0],         
         h_val[4][0],         
         h_val[5][0],           
         h_val[6][0],           
         h_val[7][0];
    //fill A_matrix
    A << A_val[0][0],    A_val[0][1],    A_val[0][2],    A_val[0][3],    A_val[0][4],    A_val[0][5],    A_val[0][6],    A_val[0][7],
         A_val[1][0],    A_val[1][1],    A_val[1][2],    A_val[1][3],    A_val[1][4],    A_val[1][5],    A_val[1][6],    A_val[1][7],
         0,              0,              1,              0,              0,              0,              0,              0,         
         0,              0,              0,              1,              0,              0,              0,              0,         
         0,              0,              0,              0,              1,              0,              0,              0,         
         0,              0,              0,              0,              0,              1,              0,              0,         
         0,              0,              0,              0,              0,              0,              1,              0,         
         0,              0,              0,              0,              0,              0,              0,              1;
    //fill B_matrix
    B << 1,              0,              0,              0,              0,              0,              0,              0,
         0,              1,              0,              0,              0,              0,              0,              0,
         0,              0,              1,              0,              0,              0,              0,              0,
         0,              0,              0,              1,              0,              0,              0,              0,         
         0,              0,              0,              0,              1,              0,              0,              0,         
         0,              0,              0,              0,              0,              1,              0,              0,         
         0,              0,              0,              0,              0,              0,              1,              0,         
         0,              0,              0,              0,              0,              0,              0,              1;
    //fill C_matrix
    C << C_val[0][0],    C_val[0][1],    C_val[0][2],    C_val[0][3],    C_val[0][4],    C_val[0][5],    C_val[0][6],    C_val[0][7],
         C_val[1][0],    C_val[1][1],    C_val[1][2],    C_val[1][3],    C_val[1][4],    C_val[1][5],    C_val[1][6],    C_val[1][7],
         0,              0,              1,              0,              0,              0,              0,              0,         
         0,              0,              0,              1,              0,              0,              0,              0,         
         0,              0,              0,              0,              1,              0,              0,              0,         
         0,              0,              0,              0,              0,              1,              0,              0,         
         0,              0,              0,              0,              0,              0,              1,              0,         
         0,              0,              0,              0,              0,              0,              0,              1;
    pc.printf("\t done\r\n");
    
    MatrixXf P_1(P);
	
    /*-----------calc-----------*/
    pc.printf("calc start");
    /*step1*/
    //prexhat
    MatrixXf prexhat(f);
    pc.printf("\t done");
    //preP
    MatrixXf preP = A*P_1*A.transpose()+pow(sigma_v,2)*B;
    pc.printf("\t done");

    /*step2*/
    //kalman gain g
    MatrixXf Q = preP*C;
    MatrixXf q = C.transpose()*Q+pow(sigma_w,2)*E;
    MatrixXf g = Q*q.inverse();
    pc.printf("\t done");
    MatrixXf xhat = prexhat + g*(y-h);
    pc.printf("\t done");
    MatrixXf P = (E - g*C.transpose())*preP;
    pc.printf("\t done\r\n");
    
    /*-----------draw-----------*/
    pc.printf("give param");
    roll = 1;//xhat.getNumber(0,0);
    pc.printf("\t done");
    pitch = 2;//xhat.getNumber(1,0);
    pc.printf("\t done\r\n");

    pc.printf("draw result \t");
    pc.printf("roll:%f \t pitch:%f",roll ,pitch);
    pc.printf("\t done\r\n");
    }
}
