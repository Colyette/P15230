

#include "IMU.h"
#include <stdio.h>

IMU::IMU(){
    printf("default IMU contructor\n");
}

IMU::IMU(int e_i2c_bus_handler) {
    dev_handle = e_i2c_bus_handler;
    printf("Created IMU module with dev handler\n");
}

IMU::~IMU(){
    printf("IMU destructor\n");
}
