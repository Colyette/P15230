#ifndef _IMU_H_
#define _IMU_H_

#include "Adafruit_LSM303.h"    // compass and accelerameter library/interface
#include "Adafruit_BMP085.h"         // barometer library/interface
#include <pthread.h>    //for i2c dev mutex

#define COMPASS_ADDRESS         (0x1E)  //address of compass, same as gy-80
#define LINEAR_ACCEL_ADDRESS    (0x19)

class IMU {
public:
    IMU(int e_i2c_bus_handler,pthread_mutex_t* e_dev_handle_mutex);
    //needs the device handler to access Linux driver's bus

    IMU(); //TODO disable the calling of default constructor

    ~IMU();
    
    //gets the 3 DoF for acceleration, pass in datastruct
    int getAccelerationValues();
    
    //gets the 3 DoF for orientation, pass in datastruct
    int getCompassValues();
    
    //get the altitude
    int getAltitude();
    
    //mag readings
    int16_t c_x;
    int16_t c_y;
    int16_t c_z;
    //accelerometer readings
    int16_t a_x;
    int16_t a_y;
    int16_t a_z;
    //baro readings
    float temp;
    float alt_m;
    int32_t slPressure;
    float r_alt_m;
    float c_base_alt;
    float avgBaro;
    //TODO store first baro altitude calculation
    
    
    
    
    float heading;
private:
    
    pthread_mutex_t* dev_handle_mutex_ptr;
    
    int dev_handle; //handler for I2C bus
    
    Adafruit_LSM303 com_acc; //compass and accelerameter module
    
    Adafruit_BMP085 baro;        //barometer module
};

#endif //_IMU_H_
