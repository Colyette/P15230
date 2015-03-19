

#include "IMU.h"
#include <stdio.h>

IMU::IMU(){
    printf("default IMU contructor\n");
}

IMU::IMU(int e_i2c_bus_handler) {
    dev_handle = e_i2c_bus_handler;
    printf("Created IMU module with dev handler\n");
    
    //TODO init LSM303 Module
    //com_acc = Adafruit_LSM303();        //init LSM303 class Module
    com_acc.set_dev_handle(dev_handle); // pass dev handle
    if ( com_acc.begin() ) {
        printf("LSM303 module configured\n");
    }else {
        printf("Counldn't configure LSM303\n");
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

int IMU::getCompassValues(){
    com_acc.readComp(); //read values from component
    //assign locally
    c_x = com_acc.magData.x;
    c_y= com_acc.magData.y;
    c_z = com_acc.magData.z;
    return 1;//TODO
}