#include <iostream>
#include <fstream>
#include <wiringPi.h>
#include <stdio.h>
#include <cmath>
#include "libSensor.h"

using namespace std;

#define PI 3.14159265
#define G  60.8577
#define gyroScale 131
#define DT 0.01  

ofstream fout("date.csv");

const int pin[5] = {0, 4, 17, 22, 27};//v-r- g_st-g-dr
const int butPin = 5;
int c;
//structura de date in care le colectez
struct data
{
    int8_t x,y,z,a,b,gx,gy,gz;
    int time;
} date[15000];
// processing variables
double arx, ary, arz, grx, gry, grz,gsx,gsy,gsz, rx, ry, rz;
double baseGyroX,baseGyroY,baseGyroZ;

//Used by Kalman Filters
float Q_angle  =  0.01;
float Q_gyro   =  0.0003;
float R_angle  =  0.01;
float x_bias = 0;
float y_bias = 0;
float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
float KFangleX = 0.0;
float KFangleY = 0.0;

float kalmanFilterX(float accAngle, float gyroRate);
float kalmanFilterY(float accAngle, float gyroRate);


float kalmanFilterX(float accAngle, float gyroRate){
    float  y, S;
    float K_0, K_1;

    KFangleX += DT * (gyroRate - x_bias);

    XP_00 +=  - DT * (XP_10 + XP_01) + Q_angle * DT;
    XP_01 +=  - DT * XP_11;
    XP_10 +=  - DT * XP_11;
    XP_11 +=  + Q_gyro * DT;

    y = accAngle - KFangleX;
    S = XP_00 + R_angle;
    K_0 = XP_00 / S;
    K_1 = XP_10 / S;

    KFangleX +=  K_0 * y;
    x_bias  +=  K_1 * y;
    XP_00 -= K_0 * XP_00;
    XP_01 -= K_0 * XP_01;
    XP_10 -= K_1 * XP_00;
    XP_11 -= K_1 * XP_01;

    return KFangleX;
}


float kalmanFilterY(float accAngle, float gyroRate){
    float  y, S;
    float K_0, K_1;


    KFangleY += DT * (gyroRate - y_bias);

    YP_00 +=  - DT * (YP_10 + YP_01) + Q_angle * DT;
    YP_01 +=  - DT * YP_11;
    YP_10 +=  - DT * YP_11;
    YP_11 +=  + Q_gyro * DT;

    y = accAngle - KFangleY;
    S = YP_00 + R_angle;
    K_0 = YP_00 / S;
    K_1 = YP_10 / S;

    KFangleY +=  K_0 * y;
    y_bias  +=  K_1 * y;
    YP_00 -= K_0 * YP_00;
    YP_01 -= K_0 * YP_01;
    YP_10 -= K_1 * YP_00;
    YP_11 -= K_1 * YP_01;

    return KFangleY;
}
////////////////////////////////////////////////////////////////

int main()
{

    wiringPiSetupGpio();

//senzorul:

    Sensor xlr8;
    perror("Init gyro");

//leduri:

    for(int i=1; i<=4; i++)
        pinMode(pin[i],OUTPUT);
//button:

    pinMode(butPin,INPUT);
    pullUpDnControl(butPin, PUD_UP);


//    fout << "Accel X,Accel Y,Accel Z,\n";

    int8_t  x,y,z,a,b;
    int stare=-1,butt,firstMillis,time2Print;


    while(1)
    {
        if(!digitalRead(butPin))
        {
            stare*=-1;
            butt=digitalRead(butPin);
            while(!butt)
                butt=digitalRead(butPin);
        }

        if(stare==1)
        {
            cout<<"begin"<<'\n';
            //led de avertisment
            digitalWrite(pin[1], HIGH); //
            delay(500);       //
            digitalWrite(pin[1], LOW);  //
            
            //valori initiale:
            for(int i=1; i<=10; i++)
            {
                baseGyroX+=xlr8.getGyroX();
                baseGyroY+=xlr8.getGyroY();
                baseGyroZ+=xlr8.getGyroZ();
                delay(100);
            }
            baseGyroX/=10;
            baseGyroY/=10;
            baseGyroZ/=10;

            fout<<"Time,AccelerationX,AccelerationY,AccelerationZ,ProcessedAccelerationX,ProcessedAccelerationY,ProcessedAccelerationZ,RotationX,RotationY,RotationZ"<<'\n';
            firstMillis = millis();
            c=1;
            while( millis() - firstMillis <= 6000 ) {
            //while (1){
                cout<<"Esantion: "<<c<<'\n';

                date[c].x= xlr8.getAccelX();
                date[c].y= xlr8.getAccelY();
                date[c].z= xlr8.getAccelZ();
                date[c].a= xlr8.getAngleX();
                date[c].b= xlr8.getAngleY();
                date[c].gx=xlr8.getGyroX();
                date[c].gy=xlr8.getGyroY();
                date[c].gz=xlr8.getGyroZ();
                date[c].time = millis()- firstMillis;
///////////////////////////////////////////////CODUL DE PROBA:

                //baseGyroX=0;
                //baseGyroY=0;
                //baseGyroZ=0;

                gsx=(date[c].gx-baseGyroX)/gyroScale;
                gsy=(date[c].gy-baseGyroY)/gyroScale;
                gsz=(date[c].gz-baseGyroZ)/gyroScale;

                //cout<<" gsx: "<<gsx<<" gsy: "<<gsy<<" gsz: "<<gsz<<'\n';

                if (c == 1)
                {
                    grx = date[c].a;
                    gry = date[c].b;
                    grz = 0 ;
                }
                else
                {
                    double timeStep = (date[c].time -date[c-1].time)/1000.0;

                    //grx = timeStep * gsx + rx;
                    //gry = timeStep * gsy + ry;
                    //grz = timeStep * gsz + rz;

                    grx = timeStep * gsx;
                    gry = timeStep * gsy;
                    grz = timeStep * gsz + rz;
                    /* 
                    grx+= rx;
                    gry+= ry;
                    grz+= rz;

                    rx = (0.9 * grx) + (0.1 * date[c].a);
                    ry = (0.9 * gry) + (0.1 * date[c].b);
                    rz = grz;

                    float kalmanY = kalmanFilterY(date[c].b, grx);    
                    float kalmanX = kalmanFilterX(date[c].a, gry);
                    */
                    rx = kalmanFilterX(date[c].a, grx);
                    ry = kalmanFilterY(date[c].b, gry);
                    rz = grz;

                    //cout<<" grx: "<<grx<<" gry: "<<gry<<" grz: "<<grz<<'\n';
                    //cout<<" rx: "<<rx<<" ry: "<<ry<<" rz: "<< rz <<" "<<'\n';
                    //cout<<"kalmanX: "<< kalmanX<<"  kalmanY: "<<kalmanY<<'\n';

                    rx*=PI/180;
                    ry*=PI/180;
                    rz*=PI/180;

                    double Rx[3][3]= {{1,0,0},
                                     {0,cos(ry),(-1)*sin(ry)},
                                     {0,sin(ry),     cos(ry)}
                    };
                    double Ry[3][3]= {{cos(rx),0,sin(rx)},
                                      {0,1,0},
                                      {(-1)*sin(rx),0,cos(rx)}
                    };
                    double Rz[3][3]= {{cos(rz), (-1) * sin(rz), 0},
                                      {sin(rz), cos(rz), 0},
                                      {0, 0, 1}
                    };

                    rx*=180/PI;
                    ry*=180/PI;
                    rz*=180/PI;

                    //produsul matricilor de rotatie:
                    double  R[3][3]= {{0,0,0},{0,0,0},{0,0,0}};
                    double R1[3][3]= {{0,0,0},{0,0,0},{0,0,0}};
                    //xlr8- acceleratii 'rotite', a- acceleratii preluate
                    double Xlr8[3]= {0,0,0};
                    double a[3]= {date[c].x,date[c].y,date[c].z} ;

                    //for(int i=0;i<=2;i++)
                    //  cout<<"a"<<i<<": "<<a[i]<<" ";
                    //cout<<'\n';

                    for (int i = 0; i < 3 ; i = i + 1)
                    {
                        for (int j = 0; j < 3 ; j = j + 1)
                        {
                            for (int k = 0; k < 3 ; k = k + 1)
                            {
                                R1[i][j] = R1[i][j] + Rz[i][k] * Rx[k][j];
                            }
                        }
                    }

                    for (int i = 0; i < 3 ; i = i + 1)
                    {
                        for (int j = 0; j < 3 ; j = j + 1)
                        {
                            for (int k = 0; k < 3 ; k = k + 1)
                            {
                                R[i][j] = R[i][j] + R1[i][k] * Ry[k][j];
                            }
                        }
                    }
                    for(int i=0; i<3; i++)
                    {
                        for(int j=0; j<3; j++)
                        {
                            Xlr8[i] += R[i][j]* a[j];
                        }
                    }

                    /*for(int i=0;i<=2;i++)
                      cout<<"ar"<<i<<": "<<Xlr8[i]<<" ";
                    cout<<'\n';
                    */
///////////////////////////////////////////////scrierea in fisier
//
//
                    fout<<date[c].time<<",";
                    fout<< static_cast<int16_t>(date[c].x)<<",";
                    fout<< static_cast<int16_t>(date[c].y)<<",";
                    fout<< static_cast<int16_t>(date[c].z)<<",";
                    fout<< Xlr8[0]<<",";
                    fout<< Xlr8[1]<<",";
                    fout<< Xlr8[2]<<",";
                    fout<< rx << ",";
                    fout<< ry << ",";
                    fout<< rz ;
                    fout<< '\n';


//        printf("Time=%d Ax=%d Ay=%d Az=%d Alpha=%d Beta=%d\n",date[c].time,date[c].x,date[c].y,date[c].z,date[c].a,date[c].b );
//        cout<<date[c].time<<","<< static_cast<int16_t>(date[c].x)<<","<< static_cast<int16_t>(date[c].y)<<","<< static_cast<int16_t>(date[c].z) <<","<< static_cast<int16_t>(date[c].a)<<","<< static_cast<int16_t>(date[c].b)<<'\n';
//////////////////////////////////////////////////////////////////

                }
                delay(6);
                c++;
            }
            cout<< (millis()- firstMillis)/(c+1)<<'\n';

            cout<<"Getting Data Done"<<'\n'<<"Processin'.."<<'\n';
            //process();
            cout<<"Done."<<'\n';

            //cod inutil dar fun//////////////////////////////
            for(int j=1; j<6; j++)      //
            {
                //  for(int i=1;i<=4;i++)     //
                digitalWrite(pin[1], HIGH); //
                delay(500);       //
                //  for(int i=1;i<=4;i++)     //
                digitalWrite(pin[1], LOW);  //
                delay(500);       //
            }         //
            //////////////////////////////////////////////////


            return 0;
        }
    }
}

