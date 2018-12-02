#include "mbed.h"
#include "MPU6050.h"
#include "math.h"
#include "Eigen/Dense.h"

#define M_PI 3.14159265358979323846
#define N 3

//debug matrix
#define PRINT_MATRIX(MATRIX)			\
  std::printf(#MATRIX "_matrix\r\n");		\
  for (int i = 0; i < MATRIX.rows(); ++i)	\
    {						\
      for (int j = 0; j < MATRIX.cols(); ++j)	\
	{					\
	  std::printf("%.3f\t", MATRIX(i, j));	\
	}					\
      std::printf("\r\n");			\
    }						\

using namespace Eigen;
Serial pc(USBTX, USBRX);
MPU6050 mpu(p28,p27);   //sda,scl
Timer t1;

int main()
{
  pc.printf("\r\n---------------------start program---------------------");
  pc.printf("\r\n");

  t1.start();
  float gyro[N]={};
  float accel[N]={};
  mpu.getGyro(gyro);
  mpu.getAccelero(accel);
  float sigma_v = 0, sigma_w = 0.5;
  float pitch, roll;
  float thetahat_p[2]={1,1};
  float thetahat_r[2]={1,1};
  float prethetahat_p[2]={1,1};
  float prethetahat_r[2]={1,1};
  float W_x[2]={1,1};
  float W_y[2]={1,1};
  float W_z[2]={1,1};
  float What_x[2]={1,gyro[0]};
  float What_y[2]={1,gyro[1]};
  float What_z[2]={1,gyro[2]};
  float preWhat_x[2]={1,1};
  float preWhat_y[2]={1,1};
  float preWhat_z[2]={1,1};
  float a_x[2]={1,1};
  float a_y[2]={1,1};
  float a_z[2]={1,1};
  float ahat_x[2]={1,accel[0]};
  float ahat_y[2]={1,accel[1]};
  float ahat_z[2]={1,accel[2]};
  float preahat_x[2]={1,1};
  float preahat_y[2]={1,1};
  float preahat_z[2]={1,1};
  float y_val[8][1]={};
  float f_val[8][1]={};
  float h_val[8][1]={};
  float A_val[8][8]={};
  float C_val[8][8]={};

  MatrixXf y(8,1);
  MatrixXf f(8,1);
  MatrixXf h(8,1);
  MatrixXf A(8,8);
  MatrixXf B(8,8);
  MatrixXf C(8,8);
  MatrixXf Identity = MatrixXf::Identity(8,8);
  // PRINT_MATRIX(Identity);
  /*
  MatrixXf SigmaV(8,1);  //bunsan
  SigmaV << 0,
            0,
            0,         
            0,         
            0,         
            0,//pow(0.0013684641,2),           
            0,//pow(0.0012666226,2),           
            0;//pow(0.003095776,2);
  // PRINT_MATRIX(SigmaV);
  */  
  MatrixXf P = Identity;
  //P => P_1
  MatrixXf P_1 = P;
  // PRINT_MATRIX(P_1);
  B = Identity;
  
  while(true)
    {
    /*-----------update-----------*/
    //timer
    double T_s = t1.read();
    t1.reset();

    //pass param
    thetahat_p[0]=thetahat_p[1];
    thetahat_r[0]=thetahat_r[1];
    prethetahat_p[0]=prethetahat_p[1];
    prethetahat_r[0]=prethetahat_r[1];
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
    ahat_x[0]=ahat_x[1];
    ahat_y[0]=ahat_y[1];
    ahat_z[0]=ahat_z[1];
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
	
    /*-----------calc-----------*/
    /*step1*/
    //f_val
    f_val[0][0] = thetahat_p[0]+(What_y[0]*cos(thetahat_r[0])-What_z[0]*sin(thetahat_r[0]))*T_s;
    f_val[1][0] = thetahat_r[0]+(What_x[0]+(What_y[0]*sin(thetahat_r[0])+What_z[0]*cos(thetahat_r[0]))*tan(thetahat_p[0]))*T_s;
    f_val[2][0] = What_x[0];
    f_val[3][0] = What_y[0];
    f_val[4][0] = What_z[0];
    f_val[5][0] = ahat_x[0];
    f_val[6][0] = ahat_y[0];
    f_val[7][0] = ahat_z[0];
    //fill f_matrix
    f << f_val[0][0],
         f_val[1][0],
         f_val[2][0],         
         f_val[3][0],         
         f_val[4][0],         
         f_val[5][0],           
         f_val[6][0],           
         f_val[7][0];
    // PRINT_MATRIX(f);
    
    //prexhat
    MatrixXf prexhat = f;
    // PRINT_MATRIX(prexhat);
    //fill prexhat param
    prethetahat_p[1] = prexhat(0,0);
    prethetahat_r[1] = prexhat(1,0);
    preWhat_x[1] = prexhat(2,0);
    preWhat_y[1] = prexhat(3,0);
    preWhat_z[1] = prexhat(4,0);
    preahat_x[1] = prexhat(5,0);
    preahat_y[1] = prexhat(6,0);
    preahat_z[1] = prexhat(7,0);

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
    //fill A_matrix
    A << A_val[0][0],    A_val[0][1],    A_val[0][2],    A_val[0][3],    A_val[0][4],    A_val[0][5],    A_val[0][6],    A_val[0][7],
         A_val[1][0],    A_val[1][1],    A_val[1][2],    A_val[1][3],    A_val[1][4],    A_val[1][5],    A_val[1][6],    A_val[1][7],
         0,              0,              1,              0,              0,              0,              0,              0,         
         0,              0,              0,              1,              0,              0,              0,              0,         
         0,              0,              0,              0,              1,              0,              0,              0,         
         0,              0,              0,              0,              0,              1,              0,              0,         
         0,              0,              0,              0,              0,              0,              1,              0,         
         0,              0,              0,              0,              0,              0,              0,              1;
    // PRINT_MATRIX(A);
    
    //C_val
    C_val[0][0] = 0;
    C_val[0][1] = 0;
    C_val[0][2] = 0;
    C_val[0][3] = 0;
    C_val[0][4] = 0;
    C_val[0][5] = -1 * sqrt( pow(preahat_y[1],2)+pow(preahat_z[1],2) ) / ( pow(preahat_x[1],2)+pow(preahat_y[1],2)+pow(preahat_z[1],2) );
    C_val[0][6] = preahat_x[1]*preahat_y[1] / (sqrt( pow(preahat_y[1],2)+pow(preahat_z[1],2) ) * ( pow(preahat_x[1],2)+pow(preahat_y[1],2)+pow(preahat_z[1],2) ));
    C_val[0][7] = preahat_x[1]*preahat_z[1] / (sqrt( pow(preahat_y[1],2)+pow(preahat_z[1],2) ) * ( pow(preahat_x[1],2)+pow(preahat_y[1],2)+pow(preahat_z[1],2) ));
    C_val[1][0] = 0;
    C_val[1][1] = 0;
    C_val[1][2] = 0;
    C_val[1][3] = 0;
    C_val[1][4] = 0;
    C_val[1][5] = 0;
    C_val[1][6] = preahat_z[1] / ( pow(preahat_y[1],2)+pow(preahat_z[1],2) );
    C_val[1][7] = -1 * preahat_y[1] / ( pow(preahat_y[1],2)+pow(preahat_z[1],2) );
    //fill C_matrix
    C << C_val[0][0],    C_val[0][1],    C_val[0][2],    C_val[0][3],    C_val[0][4],    C_val[0][5],    C_val[0][6],    C_val[0][7],
         C_val[1][0],    C_val[1][1],    C_val[1][2],    C_val[1][3],    C_val[1][4],    C_val[1][5],    C_val[1][6],    C_val[1][7],
         0,              0,              1,              0,              0,              0,              0,              0,         
         0,              0,              0,              1,              0,              0,              0,              0,         
         0,              0,              0,              0,              1,              0,              0,              0,         
         0,              0,              0,              0,              0,              1,              0,              0,         
         0,              0,              0,              0,              0,              0,              1,              0,         
         0,              0,              0,              0,              0,              0,              0,              1;
    // PRINT_MATRIX(C);
    
    //preP
    MatrixXf preP = A*P_1*A.transpose()+sigma_v*B;
    // PRINT_MATRIX(preP);

    /*step2*/
    //kalman gain g
    MatrixXf Q = preP*C.transpose();
    // PRINT_MATRIX(Q);
    MatrixXf q = C*Q+pow(sigma_w,2)*Identity;
    // PRINT_MATRIX(q);
    MatrixXf g = Q*q.inverse();
    // PRINT_MATRIX(g);
    
    //y_val
    y_val[0][0] = -1*atan2(a_x[1],sqrt(pow(a_y[1],2)+pow(a_z[1],2)));
    y_val[1][0] = atan2(a_y[1],a_z[1]);
    y_val[2][0] = W_x[1];
    y_val[3][0] = W_y[1];
    y_val[4][0] = W_z[1];
    y_val[5][0] = a_x[1];
    y_val[6][0] = a_y[1];
    y_val[7][0] = a_z[1];
    //fill y_matrix
    y << y_val[0][0],
         y_val[1][0],
         y_val[2][0],        
         y_val[3][0],       
         y_val[4][0],         
         y_val[5][0],           
         y_val[6][0],           
         y_val[7][0];
    // PRINT_MATRIX(y);
    
    //h_val
    h_val[0][0] = -1*atan2(preahat_x[1],sqrt(pow(preahat_y[1],2)+pow(preahat_z[1],2)));
    h_val[1][0] = atan2(preahat_y[1],preahat_z[1]);
    h_val[2][0] = preWhat_x[1];
    h_val[3][0] = preWhat_y[1];
    h_val[4][0] = preWhat_z[1];
    h_val[5][0] = preahat_x[1];
    h_val[6][0] = preahat_y[1];
    h_val[7][0] = preahat_z[1];
    //fill h_matrix
    h << h_val[0][0],
         h_val[1][0],
         h_val[2][0],         
         h_val[3][0],         
         h_val[4][0],         
         h_val[5][0],           
         h_val[6][0],           
         h_val[7][0];
    // PRINT_MATRIX(h);

    //xhat
    MatrixXf xhat = prexhat + g*(y-h);
    // PRINT_MATRIX(xhat);
    //fill xhat param
    thetahat_p[1] = xhat(0,0);
    thetahat_r[1] = xhat(1,0);
    What_x[1] = xhat(2,0);
    What_y[1] = xhat(3,0);
    What_z[1] = xhat(4,0);
    ahat_x[1] = xhat(5,0);
    ahat_y[1] = xhat(6,0);
    ahat_z[1] = xhat(7,0);

    //P
    MatrixXf P = (Identity - g*C)*preP;
    // PRINT_MATRIX(P);
    P_1 = P;
    // PRINT_MATRIX(P_1);
    
    /*-----------draw-----------*/
    // pc.printf("give param");
    roll = 1;//xhat.getNumber(0,0);
    // pc.printf("\t done");
    pitch = 2;//xhat.getNumber(1,0);
    // pc.printf("\t done\r\n");

    
    // pc.printf("draw result \t");
    // pc.printf("kalman: Roll:%.2f \t Pitch:%.2f",roll,pitch);
    // // pc.printf("roll:%f \t pitch:%f",roll ,pitch);
    // pc.printf("\t done\r\n");
    }
}
