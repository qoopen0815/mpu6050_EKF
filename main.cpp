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
    Matrix C(2,8);
    
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
        double dt = t1.read();
        t1.reset();
        //mpu
        mpu.getGyro(gyro);
        mpu.getAccelero(accel);

        /*-----------calc-----------*/
        xprehat << 

        /*-----------draw-----------*/
//      pc.printf("EKF: Roll:%.4f \t Pitch:%.4f",Roll,Pitch);
        pc.printf("\r\n");
        //wait(0.1);
    }
}