/**
 * \file IMU.cpp
 * \brief The IMU interface with separate drivers per
 * i2c addressed component
 * \author Alyssa Colyette
 */
#include "IMU.h"
#include <stdio.h>
#include <math.h> // for heading calculations
IMU::IMU(){
    printf("default IMU contructor\n");
}

/**
 * \brief starts all of the modules
 */
IMU::IMU(int e_i2c_bus_handler,std::recursive_mutex* e_dev_handle_mutex ) {
    dev_handle = e_i2c_bus_handler;
    printf("IMU::IMU: Created IMU module with dev handler\n");
    dev_handle_mutex_ptr = e_dev_handle_mutex;
    
    //LSM303 Module module setup
    com_acc.set_dev_handle(dev_handle); // pass dev handle
    com_acc.set_dev_mutex(dev_handle_mutex_ptr);//pass dev handle mutex pointer
    if ( com_acc.begin() ) {
        printf("IMU::IMU: LSM303 module configured\n");
    }else {
        printf("IMU::IMU: Couldn't configure LSM303\n");
    }
    
    //Baro module setup
    baro.set_dev_handle(dev_handle); //pass dev handle
    baro.set_dev_mutex(dev_handle_mutex_ptr); //pass the dev handle mutex pointer
    if (baro.begin()) {
        printf("IMU::IMU: BMP180 module configued\n");
        c_base_alt = baro.base_alt;
    } else {
        printf("IMU::IMU: Couldn't configure BMP180\n");
    }
    
    //Gyro module setup
    gyro.set_dev_handle(dev_handle); //pass dev handle
    gyro.set_dev_mutex(dev_handle_mutex_ptr); //pass the dev handle mutex pointer
    if (gyro.begin()) {
        printf("IMU::IMU: L3GD20 module configued\n");
    } else {
        printf("IMU::IMU: Couldn't configure L3GD20\n");
    }
}

IMU::~IMU(){
    printf("IMU destructor\n");
}


int IMU::getAccelerationValues(){
    com_acc.readAccel(); //read values from component
    //assign locally
    a_x = com_acc.accelData.x;
    a_y= com_acc.accelData.y;
    a_z = com_acc.accelData.z;
    return 1; //TODO
}

bool IMU::getCompassValues(){
    if ( !com_acc.readComp()) { //read values from component
        printf("IMU::getCompassValues(): error on read\n");
        return false;
    }
    
    //assign locally
    c_x = com_acc.magData.x;
    c_y= com_acc.magData.y;
    c_z = com_acc.magData.z;
    
    //calc heading
    float Pi = 3.14159;
    
    // Calculate the angle of the vector y,x
    heading = (atan2(c_y,c_x) * 180) / Pi;
    //conv to 360 deg (+)
    if (heading <0) {
        heading = heading +360;
    }
    
    return true;//TODO
}

int IMU::getAltitude(){
   // printf("Temperature = %f *C\n",baro.readTemperature());
    
    
//    Serial.print("Pressure = ");
//    Serial.print(bmp.readPressure());
//    Serial.println(" Pa");
    temp = baro.readPressure();
    //printf("Pressure = %d Pa\n",temp );
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
//    Serial.print("Altitude = ");
//    Serial.print(bmp.readAltitude());
//    Serial.println(" meters");
    alt_m = baro.readAltitude();
    //printf("Altitude = %f meters\n", alt_m);
    
//    Serial.print("Pressure at sealevel (calculated) = ");
//    Serial.print(bmp.readSealevelPressure());
//    Serial.println(" Pa");
    slPressure = baro.readSealevelPressure();
    //printf("Pressure at sealevel (calculated) = %d Pa\n", slPressure );
    
    // you can get a more precise measurement of altitude
    // if you know the current sea level pressure which will
    // vary with weather and such. If it is 1015 millibars
    // that is equal to 101500 Pascals.
//    Serial.print("Real altitude = ");
//    Serial.print(bmp.readAltitude(101500));
//    Serial.println(" meters");
    r_alt_m = baro.readAltitude(101500);
    //printf("Real altitude = %f meters\n",  r_alt_m);
    
    //Serial.println();
    //delay(500);
    
    return 1;
}

int IMU::getGryoValues(){
    gyro.read();
    g_x = gyro.data.x;
    g_y = gyro.data.y;
    g_z = gyro.data.z;
    return 1;
}