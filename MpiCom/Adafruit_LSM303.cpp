/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
/***
 * \brief Modified to work on Raspberry Pi using Linux's userspace I2C driver
 * \author Alyssa Colyette
 */

#include "Adafruit_LSM303.h"
#include <stdio.h>
#include <linux/i2c-dev.h>  //linux userspace I2C driver
#include <sys/ioctl.h>      //opens posix compliant drivers
#include <errno.h>          //reading error codes delivered by bad io
#include <fcntl.h>          //read, write file handler
#include <unistd.h>         //constants for posix compliance


/**
 * \brief default constructor
 */
Adafruit_LSM303::Adafruit_LSM303(){
    dev_handle = -1; // not initilized yet
    printf("Adafruit_LSM303 class instantiated\n");
}

/**
 * \brief Needs to be set before any sensor readings
 */
void Adafruit_LSM303::set_dev_handle(int e_dev_handle){
    dev_handle = e_dev_handle;
}

/**
 *  \brief initilizes the Accelerometer and compass modules
 */
bool Adafruit_LSM303::begin()
{
    //Wire.begin();

    uint8_t temp[2];
    int ret;
    
  // Enable the magnetometer, continous mode
    if ( !write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00)) {
        printf("Couldn't enable magnetometer\n");
        return false;
    }
    ret = readBytes(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_MR_REG_M, temp,1);
    printf("Wrote 0x%x for Mag config,ret %d\n",temp[0],ret);
  
    
    // Enable the accelerometer
    // 0x20 -> Normal/low-power mode (10Hz), 0x7 ->x,y,z axis enabled
    if ( !write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27) ) {
        printf("Couldn't enable accelerometer\n");
        return false;
    }
    readBytes(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_CTRL_REG1_A, temp, 1);
    printf("Wrote 0x%x for Accel config\n",temp[0]);

    
    printf("accelerometer and compass instantiated\n");
  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303::readAccel()
{
    uint8_t buffer[6]; //6 byte butter to store read data
    uint8_t cmd;
  // Read the accelerometer
  //Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
  //Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  //Wire.endTransmission();
  //Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);
    // Wait around until enough data is available
    //while (Wire.available() < 6);
    
    //asserting MSB for multiple byte read
    cmd = (LSM303_REGISTER_ACCEL_OUT_X_L_A |0x80);
    readBytes((uint8_t)LSM303_ADDRESS_ACCEL, cmd,buffer, (uint8_t)6);
    //assign read data
    
    uint8_t xlo = buffer[0];// Wire.read();
    uint8_t xhi = buffer[1];//Wire.read();
    uint8_t ylo = buffer[2];//Wire.read();
    uint8_t yhi = buffer[3];//Wire.read();
    uint8_t zlo = buffer[4];//Wire.read();
    uint8_t zhi = buffer[5];//Wire.read();
    
    
    
//    uint8_t xlo = read8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_X_L_A);
//    uint8_t xhi = read8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_X_H_A);
//    uint8_t ylo = read8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_Y_L_A);
//    uint8_t yhi = read8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_Y_H_A);
//    uint8_t zlo = read8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_Z_L_A);
//    uint8_t zhi = read8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_OUT_Z_H_A);

    
  // Shift values to create properly formed integer (low byte first)
    accelData.x = (int16_t) ( ((uint16_t)xlo | (uint16_t)(xhi << 8)) >> 4);
    accelData.y = (int16_t) ( ((uint16_t)xlo | (uint16_t)(xhi << 8)) >> 4);
    accelData.z = (int16_t) ( ((uint16_t)zlo | (uint16_t)(zhi << 8)) >> 4);
    
    //printf("accel:\tx:0x%x\ty:0x%x\tz:0x%x\n",(xlo | (xhi << 8)) >> 4,(xlo | (xhi << 8)) >> 4, (zlo | (zhi << 8)) >> 4);
    printf("accel:\tx:0x%x\ty:0x%x\tz:0x%x\n",accelData.x,accelData.y, accelData.z);
}

void Adafruit_LSM303::readComp()
{
    uint8_t buffer[6]; //6 byte butter to store read data
    // Read the magnetometer
    //Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
    //Wire.write(LSM303_REGISTER_MAG_OUT_X_H_M);
    //Wire.endTransmission();
    //Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);
    // Wait around until enough data is available
    //while (Wire.available() < 6);
    
    readBytes((uint8_t)LSM303_ADDRESS_MAG, (uint8_t)LSM303_REGISTER_MAG_OUT_X_H_M,buffer, (uint8_t)6);
    
    // Note high before low (different than accel)
    uint8_t xhi = buffer[0];//Wire.read();
    uint8_t xlo = buffer[1];//Wire.read();
    uint8_t zhi = buffer[2];//Wire.read();
    uint8_t zlo = buffer[3];//Wire.read();
    uint8_t yhi = buffer[4];//Wire.read();
    uint8_t ylo = buffer[5];//Wire.read();
    
//    //Testing with on byte at a time reading
//    uint8_t xhi = read8(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_X_H_M);
//    uint8_t xlo = read8(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_X_L_M);
//    uint8_t zhi = read8(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_Z_H_M);
//    uint8_t zlo = read8(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_Z_L_M);
//    uint8_t yhi = read8(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_Y_H_M);
//    uint8_t ylo = read8(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_OUT_Y_L_M);
//    
    // Shift values to create properly formed integer (low byte first)
    magData.x = (int16_t) ((uint16_t)xlo | (uint16_t)(xhi << 8));
    magData.y = (int16_t) ((uint16_t)ylo | (uint16_t)(yhi << 8));
    magData.z = (int16_t) ((uint16_t)zlo | (uint16_t)(zhi << 8));
    //printf("comp:\tx:0x%x\ty:0x%x\tz:0x%x\n",(xlo | (xhi << 8)),(ylo | (yhi << 8)), (zlo | (zhi << 8)));
    printf("comp:\tx:0x%x\ty:0x%x\tz:0x%x\n",magData.x,magData.y, magData.z);
    // ToDo: Calculate orientation
    magData.orientation = 0.0;
}


bool Adafruit_LSM303::setMagGain(lsm303MagGain gain)
{
    if (
        ! write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain) ) {
        return false;
    }
    return true;
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**
 * \brief writes one byte (value) to the provided i2c slave's reg.
 */
bool Adafruit_LSM303::write8(uint8_t address, uint8_t reg, uint8_t value)
{
    int written_bytes;
    int err,rec;
    uint8_t buffer[2];
  //Wire.beginTransmission(address);
    if( ioctl( dev_handle, I2C_SLAVE, address) < 0 ){
        err = errno ;
        printf( "ADXL345::write8: I2C bus cannot point to accelerameter comp of GY-80 Slave: errno %d\n",err);
        return false;
    }
    
    //Wire.write(reg);
    //Wire.write(value);
    buffer[0] = reg;    //assign reg to write to
    buffer[1] = value;  //value to write
    if ( write(dev_handle,buffer, 2 ) != 2 ){
        err = errno ;
        printf("ADXL345::writeRegister: change write register address: errno %d\n",err);
        return false;
    }
    
  
    /*
    *buffer = value;
    if ( written_bytes = (write(dev_handle,buffer, 1 ) )!= 1 ){ //write one byte
        err = errno ;
        printf("ADXL345::writeRegister: couldn't write to register: errno %d\n",err);
        return false;
    }
    */
  //Wire.endTransmission();
    return true;
}

/**
 * \brief reads one byte (value) from the provided i2c slave's reg.
 */
uint8_t Adafruit_LSM303::read8(uint8_t address, uint8_t reg)
{
    int written_bytes;
    int err,rec;
    uint8_t buffer[2];
    buffer[0] = reg; //assign reg to read from
    

  //Wire.beginTransmission(address);
    if( ioctl( dev_handle, I2C_SLAVE, address) < 0 ){
        err = errno ;
        printf( "ADXL345::read8: I2C bus cannot open device at address 0x%x: errno %d\n",address,err);
        return -1;
    }
  //Wire.write(reg);
    
    if ( write(dev_handle,buffer, 1 ) != 1 ){
        err = errno ;
        printf("ADXL345::read8: change write register address: errno %d\n",err);
        return -1;
    }
  //Wire.endTransmission();
  //Wire.requestFrom(address, (byte)1);
    if ((rec = ::read( dev_handle,buffer, 1 )) != 1  ) { // read one byte
        err = errno ;
        printf("ADXL345::read8: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return -1;
    }
    //value = Wire.read();
    
  //Wire.endTransmission();
     
    
//    reg = reg <<1;
//    if(	write( dev_handle, &reg, 1 ) != 1 ||
//       ( rec = ::read( dev_handle, &buffer, 1 )) != 1	){
//        printf( "ADXL345::read8: reg 0x%x failed, got %d\n", reg, rec ) ;
//        return false ;
//    }
  return buffer[0];
}

/**
 * \brief reads one byte (value) from the provided i2c slave's reg.
 * \param address i2c address
 * \param reg register to read read from?
 * \param buffer where the read data is stored
 * \param nBytes number of bytes to read
 */
bool Adafruit_LSM303::readBytes(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes)
{
    int written_bytes;
    int err,rec;
    //uint8_t ibuffer= *buffer;
    buffer[0] = reg; // set bit high for multi-byte reading
    
    //uint8_t ret; //return value
    
    //Wire.beginTransmission(address);
    if( ioctl( dev_handle, I2C_SLAVE, address) < 0 ){
        err = errno ;
        printf( "ADXL345::readBytes: I2C bus cannot open device at address 0x%x: errno %d\n",address,err);
        return false;
    }
    //Wire.write(reg);
    if ( write(dev_handle,buffer, 1 ) != 1 ){
        err = errno ;
        printf("ADXL345::readBytes: change write register address: errno %d\n",err);
        return false;
    }
    //Wire.endTransmission();
    //Wire.requestFrom(address, (byte)1);
    if ((rec = ::read( dev_handle,buffer, nBytes )) != nBytes  ) { // read one byte
        err = errno ;
        printf("ADXL345::readBytes: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return false;
    }
    
    
    //Wire.endTransmission();
    return true;
}

