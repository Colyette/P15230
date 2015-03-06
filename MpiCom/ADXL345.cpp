/********************************************************************************
* ADXL345 Library - ADXL345.cpp                                                 *
*                                                                               *
* Copyright (C) 2012 Anil Motilal Mahtani Mirchandani(anil.mmm@gmail.com)       *
*                                                                               *
* License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html> *
* This is free software: you are free to change and redistribute it.            *
* There is NO WARRANTY, to the extent permitted by law.                         *
*                                                                               *
*********************************************************************************/
/**
 * Modifications made to use with non-Arduino Cpp using linux i2c userspace driver
 * sourced from: https://github.com/Anilm3/ADXL345-Accelerometer/tree/master/Arduino
 * Modified by Alyssa Colyette
 */

#include "ADXL345.h"
#include <stdio.h>
#include <linux/i2c-dev.h>  //linux userspace I2C driver
#include <sys/ioctl.h>      //opens posix compliant drivers
#include <errno.h>          //reading error codes delivered by bad io
#include <fcntl.h>          //read, write file handler
#include <unistd.h>         //constants for posix compliance

//#include <Wire.h>

int ADXL345::writeRegister(uint8_t reg_addr, int nbytes, uint8_t* buffer)
{
	int written_bytes;
    int err,rec;

    //	Wire.beginTransmission(ADXL345::ADDRESS);
    if( ioctl( dev_handle, I2C_SLAVE, ADXL345::ADDRESS) < 0 ){
        err = errno ;
        printf( "ADXL345::writeRegister: I2C bus cannot point to accelerameter comp of GY-80 Slave: errno %d\n",err);
        return 0;
    }
    //	Wire.write(reg_addr);
    if ( write(dev_handle,&reg_addr, 1 ) != 1 ){
        err = errno ;
        printf("ADXL345::writeRegister: change write register address: errno %d\n",err);
        return -1;
    }
    
    //	written_bytes = Wire.write(buffer, nbytes);
    if ( written_bytes = (write(dev_handle,buffer, nbytes ) )!= nbytes ){
        err = errno ;
        printf("ADXL345::writeRegister: couldn't write to register: errno %d\n",err);
        return -1;
    }
    
//	Wire.endTransmission();
    
	return written_bytes;
}

int ADXL345::readRegister(uint8_t reg_addr, int nbytes, uint8_t* buffer )
{
	int idx = 0;
    int err,rec; //for i2c errors

//	Wire.beginTransmission(ADXL345::ADDRESS);
    if( ioctl( dev_handle, I2C_SLAVE, ADXL345::ADDRESS) < 0 ){
        err = errno ;
        printf( "ADXL345::readRegister: I2C bus cannot point to accelerameter comp of GY-80 Slave: errno %d \n",err);
        return 0;
    }
//	Wire.write(reg_addr);
    if ( write(dev_handle,&reg_addr, 1 ) != 1 ){
        err = errno ;
        printf("ADXL345::readRegister: change write register address: errno %d\n",err);
        return -1;
    }
//	Wire.endTransmission(); 
//	
//	Wire.requestFrom(ADXL345::ADDRESS, nbytes);
    if ((rec = ::read( dev_handle,buffer, nbytes )) != nbytes  ) { // sized in bytes
        err = errno ;
        printf("ADXL345::readRegister: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return -1;
    }
//	while(Wire.available() && idx < nbytes)
//	{ 
//		buffer[idx++] = Wire.read();
//	}
	
	return idx;
}

ADXL345::ADXL345(int e_dev_handle)
{
	//Wire.begin();
    dev_handle = e_dev_handle;
	zG[0] = -20;
	zG[1] =  15;
	zG[2] = -23;
}

int ADXL345::update_dev_handle (int n_dev_handle){
    dev_handle = n_dev_handle;
}

int ADXL345::begin()
{
    int returnErr;
    
	uint8_t data = 0x08;
	returnErr = writeRegister(ADXL345::POWER_CTL, 1, &data);
    
    return returnErr;
}

int ADXL345::end()
{
    int returnErr;
    
	uint8_t data = 0x00;
	returnErr = writeRegister(ADXL345::POWER_CTL, 1, &data);
    
    return returnErr;
}

//Returns error in reading
int ADXL345::read(double *x, double *y, double *z)
{
    int returnErr;
	uint8_t buffer[6];

	returnErr = readRegister(ADXL345::DATAX0, 6, buffer);
	
	*x = ((buffer[0] + (buffer[1] << 8)) - zG[0])/256.0;
	*y = ((buffer[2] + (buffer[3] << 8)) - zG[1])/256.0;
	*z = ((buffer[4] + (buffer[5] << 8)) - zG[2])/256.0;
    
    return returnErr;
}

int ADXL345::read(int *x, int *y, int *z)
{
    int returnErr;
	uint8_t buffer[6];

	readRegister(ADXL345::DATAX0, 6, buffer);
	
	*x = buffer[0] + (buffer[1] << 8);
	*y = buffer[2] + (buffer[3] << 8);
	*z = buffer[4] + (buffer[5] << 8);
    
    return returnErr;
}


int ADXL345::setRange(range_t range)
{
    int returnErr;
    
	switch(range)
	{
		case RANGE_16G:
		case RANGE_8G:
		case RANGE_4G:
		case RANGE_2G:
			returnErr = writeRegister(ADXL345::DATA_FORMAT, 1, (uint8_t *)&range);
			break;
            //TODO default range
	}
    return returnErr;
}

void ADXL345::setZeroG(double x, double y, double z)
{
	zG[0] = x*256.0;
	zG[1] = y*256.0;
	zG[2] = z*256.0;
}

void ADXL345::setZeroG(int x, int y, int z)
{
	zG[0] = x;
	zG[1] = y;
	zG[2] = z;
}
