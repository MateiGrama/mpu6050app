* Developed by Grama Matei
* Created December 2015

This program (gyro.cpp) will enable your Raspberry Pi to read the values from a MPU6050 and convert them to valuable data for interpreting the device's movement: using the Kalman filter, the rotation angles are calculated; those angles are used to 'rotate' the 3-axis-acceleration trihedral using rotation matrices, in order to obtain instantaneous sets of accelerations independent from the device's orientation.
The program uses a button and an LED to start and to mark the end of the data gathering cycle respectively. 
sample.cpp is a source made for testing the sensor using LED's.

The Kalman Filter was implemented using the fallowing tutorial:
http://ozzmaker.com/guide-interfacing-gyro-accelerometer-raspberry-pi-kalman-filter/

Compile with;
gcc -o gyro -l rt gyro.c++ -lm

Run with;
sudo ./gyro

The code was created as part of a more complex project:
https://1drv.ms/b/s!Am1FK_5miPXj3lkHl68Mt_fEzBf5
