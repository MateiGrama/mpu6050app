#include <iostream>
#include <fstream>
#include <wiringPi.h>
#include <stdio.h>
#include <cmath>
#include "libSensor.h"

using namespace std;

//ofstream fout("date.csv");

const int pin[5] = {0, 4, 17, 22, 27};//v-r- g_st-g-dr
const int butPin = 5;

int main()
{
//    ofstream fout;
//    fout.open("date.csv");

    
    wiringPiSetupGpio();

//senzorul://////////////////////////////////////////////////////

    Sensor xlr8;
    perror("Init gyro");
    
//leduri:////////////////////////////////////////////////////////

for(int i=1;i<=4;i++)
    pinMode(pin[i],OUTPUT);
//button:///////////////////////////////////////////////////////

	pinMode(butPin,INPUT);
	pullUpDnControl(butPin, PUD_UP);


//    fout << "Accel X,Accel Y,Accel Z,\n";
	
    int x,y,z,a,b,stare=-1,butt;

    while(1){
    	//cout<<digitalRead(butPin)<<'\n';
    	if(!digitalRead(butPin)){
    	  stare*=-1;
    	  butt=digitalRead(butPin);
    	  while(!butt)
    	  	butt=digitalRead(butPin);
    	}
    		
    	if(stare==1){
    		a= xlr8.getAngleX();
	    	b= xlr8.getAngleY();
	    	
	        x=xlr8.getAccelX();
	        y=xlr8.getAccelY();
	        z=xlr8.getAccelZ();
		
		if(a <-10) digitalWrite(pin[1], HIGH); else digitalWrite(pin[1], LOW);
		if(a > 10) digitalWrite(pin[2], HIGH); else digitalWrite(pin[2], LOW);
		if(b <-10) digitalWrite(pin[3], HIGH); else digitalWrite(pin[3], LOW);
		if(b > 10) digitalWrite(pin[4], HIGH); else digitalWrite(pin[4], LOW);
		 
	        printf("AngleX=%f    AngleY=%f\n", xlr8.getAngleX(),xlr8.getAngleY());
    	}
    }
}

