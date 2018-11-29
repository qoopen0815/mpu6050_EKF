#include "mbed.h"
#include "MPU6050.h"
#include "math.h"
#include "Matrix.h"

#define M_PI 3.14159265358979323846
#define N 3

Serial pc(USBTX, USBRX);
MPU6050 mpu(p28,p27);   //sda,scl
Timer t1;

int main()
{
    t1.start();
    float gyro[N];
    float accel[N];
    float prethetahat_r, prethetahat_p;
    float preWhat_x, preWhat_y, preWhat_z;
    float a_x, a_y, a_z;
    Matrix xpre;
    Matrix xpre_;
    Matrix xhat;
    Matrix xhat_;
    Matrix xprehat;
    Matrix y(2,1);
    Matrix ypre;
    Matrix yhat;
    Matrix yprehat;
    Matrix A(8,8);
    Matrix C(8,8);

    float A_val[2][8][8], C_val[2][8][8];
      
    //初期化 initialize
    for(int i = 0; i < N; i++ ) {
        gyro[i] = 0;
        accel[i] = 0;
    }
    mpu.getGyro(gyro);
    mpu.getAccelero(accel);
    
    // Fill Matrix with data.
    xhat_ << 0  << 0  << 0  << 0  << 0  << 0  << 0  << 0;

    while(true) {
        /*-----------update-----------*/
        //timer
        double T_s = t1.read();
        t1.reset();
        //mpu
        mpu.getGyro(gyro);
        mpu.getAccelero(accel);
	//A_val
	A_val[0][0][0] = 1;
	A_val[0][0][1] = (-preWhat_y*sin(prethetahat_r)-preWhat_z*cos(prethetahat_r))*T_s;
	A_val[0][0][2] = 0;
	A_val[0][0][3] = T_s*cos(prethetahat_r);
	A_val[0][0][4] = -1*T_s*sin(prethetahat_r);
	A_val[0][0][5] = 0;
	A_val[0][0][6] = 0;
	A_val[0][0][7] = 0;
	A_val[0][1][0] = ((preWhat_y*sin(prethetahat_r))/(cos(prethetahat_p)^2)+(preWhat_z*cos(prethetahat_r))/(cos(prethetahat_p)^2))*T_s;
	A_val[0][1][1] = 1+(preWhat_y*cos(prethetahat_r)-preWhat_z*sin(prethetahat_r))*T_s*tan(prethetahat_p);
	A_val[0][1][2] = T_s;
	A_val[0][1][3] = T_s*sin(prethetahat_r)*tan(prethetahat_p);
	A_val[0][1][4] = T_s*cos(prethetahat_r)*tan(prethetahat_p);
	A_val[0][1][5] = 0;
	A_val[0][1][6] = 0;
	A_val[0][1][7] = 0;
	//C_val
	C_val[0][0][0] = 0;
	C_val[0][0][1] = 0;
	C_val[0][0][2] = 0;
	C_val[0][0][3] = 0;
	C_val[0][0][4] = 0;
	C_val[0][0][5] = -1*sqrt(a_y^2)+a_z^2)/(a_x^2+a_y^2+a_z^2);
	C_val[0][0][6] = a_x*a_y/(sqrt(a_y^2+a_z^2)*(a_x^2+a_y^2+a_z^2));
	C_val[0][0][7] = a_x*a_z/(sqrt(a_y^2+a_z^2)*(a_x^2+a_y^2+a_z^2));
	C_val[0][1][0] = 0;
	C_val[0][1][1] = 0;
	C_val[0][1][2] = 0;
	C_val[0][1][3] = 0;
	C_val[0][1][4] = 0;
	C_val[0][1][5] = 0;
	C_val[0][1][6] = a_z/(a_y^2+a_z^2);
	C_val[0][1][7] = -1*a_y/(a_y^2+a_z^2);
	
        /*-----------calc-----------*/
        A << A_val[0][0][0]  << A_val[0][0][1]  << A_val[0][0][2]  << A_val[0][0][3]  << A_val[0][0][4]  << A_val[0][0][5]  << A_val[0][0][6]  << A_val[0][0][7]
	  << A_val[0][1][0]  << A_val[0][1][1]  << A_val[0][1][2]  << A_val[0][1][3]  << A_val[0][1][4]  << A_val[0][1][5]  << A_val[0][1][6]  << A_val[0][1][7]
	  << 0               << 0               << 1               << 0               << 0               << 0               << 0               << 0         
	  << 0               << 0               << 0               << 1               << 0               << 0               << 0               << 0         
	  << 0               << 0               << 0               << 0               << 1               << 0               << 0               << 0         
	  << 0               << 0               << 0               << 0               << 0               << 1               << 0               << 0         
	  << 0               << 0               << 0               << 0               << 0               << 0               << 1               << 0         
	  << 0               << 0               << 0               << 0               << 0               << 0               << 0               << 1;
	
        C << C_val[0][0][0]  << C_val[0][0][1]  << C_val[0][0][2]  << C_val[0][0][3]  << C_val[0][0][4]  << C_val[0][0][5]  << C_val[0][0][6]  << C_val[0][0][7]
	  << C_val[0][1][0]  << C_val[0][1][1]  << C_val[0][1][2]  << C_val[0][1][3]  << C_val[0][1][4]  << C_val[0][1][5]  << C_val[0][1][6]  << C_val[0][1][7]
	  << 0               << 0               << 1               << 0               << 0               << 0               << 0               << 0         
	  << 0               << 0               << 0               << 1               << 0               << 0               << 0               << 0         
	  << 0               << 0               << 0               << 0               << 1               << 0               << 0               << 0         
	  << 0               << 0               << 0               << 0               << 0               << 1               << 0               << 0         
	  << 0               << 0               << 0               << 0               << 0               << 0               << 1               << 0         
	  << 0               << 0               << 0               << 0               << 0               << 0               << 0               << 1;

        /*-----------draw-----------*/
//      pc.printf("EKF: Roll:%.4f \t Pitch:%.4f",Roll,Pitch);
        pc.printf("\r\n");
        //wait(0.1);
    }
}
