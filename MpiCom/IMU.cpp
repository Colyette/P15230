

#include "IMU.h"
#include <stdio.h>
#include <math.h> // for heading calculations
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
        printf("Couldn't configure LSM303\n");
    }
    
    //TODO init SFE_BMP180
    baro.set_dev_handle(dev_handle); //pass dev handle
    if (baro.begin()) {
        printf("BMP180 module configued\n");
    } else {
        printf("Couldn't configure BMP180\n");
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
    
    //calc heading
    float Pi = 3.14159;
    
    // Calculate the angle of the vector y,x
    heading = (atan2(c_y,c_x) * 180) / Pi;
    //conv to 360 deg (+)
    if (heading <0) {
        heading = heading +360;
    }
    
    return 1;//TODO
}

int IMU::getAltitude(){
    printf("Temperature = %f *C\n",baro.readTemperature());
    
    
//    Serial.print("Pressure = ");
//    Serial.print(bmp.readPressure());
//    Serial.println(" Pa");
    printf("Pressure = %d Pa\n",baro.readPressure() );
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
//    Serial.print("Altitude = ");
//    Serial.print(bmp.readAltitude());
//    Serial.println(" meters");
    printf("Altitude = %f meters\n",baro.readAltitude() );
    
//    Serial.print("Pressure at sealevel (calculated) = ");
//    Serial.print(bmp.readSealevelPressure());
//    Serial.println(" Pa");
    printf("Pressure at sealevel (calculated) = %d Pa\n", baro.readSealevelPressure() );
    
    // you can get a more precise measurement of altitude
    // if you know the current sea level pressure which will
    // vary with weather and such. If it is 1015 millibars
    // that is equal to 101500 Pascals.
//    Serial.print("Real altitude = ");
//    Serial.print(bmp.readAltitude(101500));
//    Serial.println(" meters");
    printf("Real altitude = %f meters\n", baro.readAltitude(101500) );
    
    //Serial.println();
    //delay(500);
    
    return 1;
}