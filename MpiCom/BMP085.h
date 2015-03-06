//
//  BMP085.h
//  
//
//  Created by Alyssa Colyette on 3/5/15.
//
//

#ifndef ____BMP085__
#define ____BMP085__

#include <stdio.h>
#include <stdint.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085
const unsigned char OSS = 0;  // Oversampling Setting

class BMP085 {
    
private:
    // Calibration values
    int ac1;
    int ac2;
    int ac3;
    unsigned int ac4;
    unsigned int ac5;
    unsigned int ac6;
    int b1;
    int b2;
    int mb;
    int mc;
    int md;
    
    // b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
    // so ...Temperature(...) must be called before ...Pressure(...).
    long b5;
    
    int dev_handle; //linux userspace I2C device handle (opened externally)
    
    BMP085(); //no default
    
    int bmp085Calibration();
    
    uint8_t bmp085Read( uint8_t address);
    
    uint16_t bmp085ReadInt(uint8_t address);
    
    uint16_t bmp085ReadUT();
    
    uint32_t bmp085ReadUP();
    
    int writeRegister(int deviceAddress, uint8_t address, uint8_t val);
    
    uint8_t BMP085::readRegister(int deviceAddress, uint8_t address);
    
    
public:
    BMP085(int e_dev_handle);
    
    // \brief updates the dev handle, just in case it was invalid during exe
    int update_dev_handle (int n_dev_handle);
    
    //calls internal calibration funct
    int setup();
    
    //timely and is required before a baro reading can be made...
    float bmp085GetTemperature(unsigned int ut);
    
    long bmp085GetPressure(unsigned long up);
    
    float BMP085::calcAltitude(float pressure){;
};

#endif /* defined(____BMP085__) */
