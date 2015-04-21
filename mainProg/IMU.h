/**
 * \file IMU.h
 * \brief Header file for the IMU interface with separate drivers per
 * i2c addressed component
 * \author Alyssa Colyette
 */
#ifndef _IMU_H_
#define _IMU_H_

#include "Adafruit_LSM303.h"    // compass and accelerameter library/interface
#include "Adafruit_BMP085.h"         // barometer library/interface
#include "Adafruit_L3GD20.h"    //gyroscop librar/interface
//#include <pthread.h>    //for i2c dev mutex
#include <mutex>        //for recursive mutex w/ unique lock

#define COMPASS_ADDRESS         (0x1E)  //address of compass, same as gy-80
#define LINEAR_ACCEL_ADDRESS    (0x19)

#define COMSTATUS_COM_ACC   (0x01)
#define COMSTATUS_BARO      (0x02)
#define COMSTATUS_GYRO      (0x03)

class IMU {
public:
    IMU(int e_i2c_bus_handler,std::recursive_mutex* e_dev_handle_mutex);
    //needs the device handler to access Linux driver's bus

    IMU(); //TODO disable the calling of default constructor

    ~IMU();
    
    //gets the 3 DoF for acceleration, pass in datastruct
    int getAccelerationValues();
    
    //gets the 3 DoF for orientation, pass in datastruct
    bool getCompassValues();
    
    //get the altitude
    int getAltitude();
    
    //get the x,y,z values of the gyroscope module
    int getGryoValues();
    
    //mag readings, updated in getAccelerationValues
    int16_t c_x;
    int16_t c_y;
    int16_t c_z;
    float heading; //heading calculated in degrees
    //accelerometer readings, updated in getCompassValues
    int16_t a_x;
    int16_t a_y;
    int16_t a_z;
    //baro readings, updated in getAltitude (except for c_base_alt, done at class creation)
    float temp;
    float alt_m;
    int32_t slPressure;
    float r_alt_m;
    float c_base_alt; //first baro reading at init established as base
    float avgBaro; //running avg of baro
    
    //Gyro readings
    float g_x;
    float g_y;
    float g_z;
    
    /**
     * \brief returns the comStatus, positive flags mean online
     */
    uint8_t getcomStatus(){return comStatus;}
    
private:
    
    std::recursive_mutex* dev_handle_mutex_ptr;
    
    int dev_handle; //handler for I2C bus
    uint8_t comStatus; //tells if the sensors are online, set at initialization
    
    //IMU module drivers
    Adafruit_LSM303 com_acc; //compass and accelerameter module
    Adafruit_BMP085 baro;        //barometer module
    Adafruit_L3GD20 gyro;   //gyroscope module
};

#endif //_IMU_H_
