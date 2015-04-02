/***************************************************
  This is a library for the L3GD20 and L3GD20H GYROSCOPE

  Designed specifically to work with the Adafruit L3GD20(H) Breakout 
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
/**
 * \brief edited for using linux userspace i2c driver for RPi
 * \modified by Alyssa Colyette
 */

#include "Adafruit_L3GD20.h"
#include <stdio.h>
#include <linux/i2c-dev.h>  //linux userspace I2C driver
#include <sys/ioctl.h>      //opens posix compliant drivers
#include <errno.h>          //reading error codes delivered by bad io
#include <fcntl.h>          //read, write file handler
#include <unistd.h>         //constants for posix compliance

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

Adafruit_L3GD20::Adafruit_L3GD20(int8_t cs, int8_t miso, int8_t mosi, int8_t clk) {
  _cs = cs;
  _miso = miso;
  _mosi = mosi;
  _clk = clk;
}

/**
 * \brief Only instantiation garanteed to work on the RPi using the 
 * linux i2c device tree driver
 */
Adafruit_L3GD20::Adafruit_L3GD20(void) {
  // use i2c
  _cs = _mosi = _miso = _clk = -1;
}

/**
 * \brief Needs to be set before any sensor readings
 */
void Adafruit_L3GD20::set_dev_handle(int e_dev_handle){
    dev_handle = e_dev_handle;
}

/**
 * \brief passed mutex for the dev handle
 */
void Adafruit_L3GD20::set_dev_mutex(std::recursive_mutex* e_dev_handle_mutex){
    dev_handle_mutex_ptr = e_dev_handle_mutex;
}


bool Adafruit_L3GD20::begin(l3gd20Range_t rng, uint8_t addr)
{
  if (_cs == -1) {
    //Wire.begin();
  } else {
      printf("Adafruit_L3GD20::begin: SPI mode not enabled\n");
      return false;
      //DISABLED
//    pinMode(_cs, OUTPUT);
//    pinMode(_clk, OUTPUT);
//    pinMode(_mosi, OUTPUT);
//    pinMode(_miso, INPUT);
//    digitalWrite(_cs, HIGH);
  }

  address = addr;
  range = rng;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = read8(L3GD20_REGISTER_WHO_AM_I);
  //Serial.println(id, HEX);
  if ((id != L3GD20_ID) && (id != L3GD20H_ID))
  {
    return false;
  }

  /* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Switch to normal mode and enable all three channels */
  write8(L3GD20_REGISTER_CTRL_REG1, 0x0F);
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Adjust resolution if requested */
  switch(range)
  {
    case L3DS20_RANGE_250DPS:
      write8(L3GD20_REGISTER_CTRL_REG4, 0x00);
      break;
    case L3DS20_RANGE_500DPS:
      write8(L3GD20_REGISTER_CTRL_REG4, 0x10);
      break;
    case L3DS20_RANGE_2000DPS:
      write8(L3GD20_REGISTER_CTRL_REG4, 0x20);
      break;
  }
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_L3GD20::read()
{ 
  uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
    uint8_t buffer[7]; //buffer to store read
    uint8_t cmd;

  if (_cs == -1) {
    //Wire.beginTransmission(address);
      
    // Make sure to set address auto-increment bit
//    Wire.write(L3GD20_REGISTER_OUT_X_L | 0x80);
//    Wire.endTransmission();
//    Wire.requestFrom(address, (byte)6);
      cmd = (L3GD20_REGISTER_OUT_X_L |0x80);
      readBytes((uint8_t)address, cmd,buffer, (uint8_t)6);
      
    // Wait around until enough data is available
    //while (Wire.available() < 6);
    
      xlo = buffer[0];//Wire.read();
      xhi = buffer[1];//Wire.read();
      ylo = buffer[2];//Wire.read();
      yhi = buffer[3];//Wire.read();
      zlo = buffer[4];//Wire.read();
      zhi = buffer[5];//Wire.read();

  } else {
      //DISABLED
      printf("Adafruit_L3GD20::read: SPI mode not enabled\n");
//    digitalWrite(_clk, HIGH);
//    digitalWrite(_cs, LOW);
//
//    SPIxfer(L3GD20_REGISTER_OUT_X_L | 0x80 | 0x40); // SPI read, autoincrement
//    delay(10);
//    xlo = SPIxfer(0xFF);
//    xhi = SPIxfer(0xFF);
//    ylo = SPIxfer(0xFF);
//    yhi = SPIxfer(0xFF);
//    zlo = SPIxfer(0xFF);
//    zhi = SPIxfer(0xFF);
//
//    digitalWrite(_cs, HIGH);
      return;
  }
  // Shift values to create properly formed integer (low byte first)
  data.x = (xlo | (xhi << 8));
  data.y = (ylo | (yhi << 8));
  data.z = (zlo | (zhi << 8));
  
  // Compensate values depending on the resolution
  switch(range)
  {
    case L3DS20_RANGE_250DPS:
      data.x *= L3GD20_SENSITIVITY_250DPS;
      data.y *= L3GD20_SENSITIVITY_250DPS;
      data.z *= L3GD20_SENSITIVITY_250DPS;
      break;
    case L3DS20_RANGE_500DPS:
      data.x *= L3GD20_SENSITIVITY_500DPS;
      data.y *= L3GD20_SENSITIVITY_500DPS;
      data.z *= L3GD20_SENSITIVITY_500DPS;
      break;
    case L3DS20_RANGE_2000DPS:
      data.x *= L3GD20_SENSITIVITY_2000DPS;
      data.y *= L3GD20_SENSITIVITY_2000DPS;
      data.z *= L3GD20_SENSITIVITY_2000DPS;
      break;
  }
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
uint8_t Adafruit_L3GD20::write8(l3gd20Registers_t reg, uint8_t value)
{
    int written_bytes;
    int err,rec;
    uint8_t buffer[2];
  if (_cs == -1) {
    // use i2c
//    Wire.beginTransmission(address);
//    Wire.write((byte)reg);
//    Wire.write(value);
//    Wire.endTransmission();
      
      //LOCK~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      if (pthread_mutex_lock(dev_handle_mutex_ptr) ){
//          printf("Adafruit_L3GD20::write8: Error locking thread\n");
//          return (-1);
//      }
      std::unique_lock<std::recursive_mutex> lck(*dev_handle_mutex_ptr);
      
      //Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
      if( ioctl( dev_handle, I2C_SLAVE, address) < 0 ){
          err = errno ;
          printf( "Adafruit_L3GD20::write8: I2C bus cannot point to barometer of IMU Slave: errno %d\n",err);
          return (-1);
      }
      buffer[0] = reg;    //assign reg to write to
      buffer[1] = value;  //value to write
      if ( write(dev_handle,buffer, 2 ) != 2 ){
          err = errno ;
          printf("Adafruit_L3GD20::write8: change write register address: errno %d\n",err);
          return (-1);
      }
      //UNLOCK~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      if (pthread_mutex_unlock( dev_handle_mutex_ptr) ) {
//          printf("Adafruit_L3GD20::write8:Error unlocking thread\n");
//          return (-1);
//      }
//
  } else {
      printf("Adafruit_L3GD20::write8: SPI mode not enabled\n");
      return (-1);
//    digitalWrite(_clk, HIGH);
//    digitalWrite(_cs, LOW);
//
//    SPIxfer(reg);
//    SPIxfer(value);
//
//    digitalWrite(_cs, HIGH);
  }
    return 1;
}

uint8_t Adafruit_L3GD20::read8(l3gd20Registers_t reg)
{
    uint8_t ret;
    int err,rec;
    uint8_t buffer[2];
    buffer[0] =reg;

  if (_cs == -1) {
    // use i2c
//    Wire.beginTransmission(address);
//    Wire.write((byte)reg);
//    Wire.endTransmission();
//    Wire.requestFrom(address, (byte)1);
//    value = Wire.read();
//    Wire.endTransmission();
      
      //LOCK
//      if (pthread_mutex_lock(dev_handle_mutex_ptr) ){
//          printf("Adafruit_L3GD20::read8: Error locking thread\n");
//          return (-1);
//      }
      std::unique_lock<std::recursive_mutex> lck(*dev_handle_mutex_ptr);
      
      if( ioctl( dev_handle, I2C_SLAVE, address) < 0 ){
          err = errno ;
          printf( "Adafruit_L3GD20::read8: I2C bus cannot point to barometer of IMU Slave: errno %d\n",err);
          return (-1);
      }
      
      if ( write(dev_handle,buffer, 1 ) != 1 ){
          err = errno ;
          printf("Adafruit_L3GD20::read8: change write register address: errno %d\n",err);
          return (-1);
      }
      
      if ((rec = ::read( dev_handle,buffer, 1 )) != 1  ) { // read one byte
          err = errno ;
          printf("Adafruit_L3GD20::read8: Couldn't read from register: errno %d rec: %d\n",err,rec);
          return -1;
      }
      ret = buffer[0];
      
      //UNLOCK~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      if (pthread_mutex_unlock( dev_handle_mutex_ptr) ) {
//          printf("Adafruit_L3GD20::read8: Error unlocking thread\n");
//          return (-1);
//      }
      

  } else {
      printf("Adafruit_L3GD20::read8: SPI mode not enabled\n");
//    digitalWrite(_clk, HIGH);
//    digitalWrite(_cs, LOW);
//
//    SPIxfer((uint8_t)reg | 0x80); // set READ bit
//    value = SPIxfer(0xFF);
//
//    digitalWrite(_cs, HIGH);
  }

  return ret;
}


/**
 * \brief reads one byte (value) from the provided i2c slave's reg.
 * \param address i2c address
 * \param reg register to read read from?
 * \param buffer where the read data is stored
 * \param nBytes number of bytes to read
 */
bool Adafruit_L3GD20::readBytes(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t nBytes)
{
    int written_bytes;
    int err,rec;
    //uint8_t ibuffer= *buffer;
    buffer[0] = reg; // set bit high for multi-byte reading
    
    //uint8_t ret; //return value
    
    //LOCK~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    if (pthread_mutex_lock(dev_handle_mutex_ptr) ){
//        printf("Adafruit_L3GD20::readBytes: Error locking thread\n");
//        return (-1);
//    }
    std::unique_lock<std::recursive_mutex> lck(*dev_handle_mutex_ptr);
    
    //Wire.beginTransmission(address);
    if( ioctl( dev_handle, I2C_SLAVE, address) < 0 ){
        err = errno ;
        printf( "Adafruit_L3GD20::readBytes: I2C bus cannot open device at address 0x%x: errno %d\n",address,err);
        return false;
    }
    //Wire.write(reg);
    if ( write(dev_handle,buffer, 1 ) != 1 ){
        err = errno ;
        printf("Adafruit_L3GD20::readBytes: change write register address: errno %d\n",err);
        return false;
    }
    //Wire.endTransmission();
    //Wire.requestFrom(address, (byte)1);
    if ((rec = ::read( dev_handle,buffer, nBytes )) != nBytes  ) { // read one byte
        err = errno ;
        printf("Adafruit_L3GD20::readBytes: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return false;
    }
    
    //UNLOCK~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//    if (pthread_mutex_unlock( dev_handle_mutex_ptr) ) {
//        printf("Adafruit_L3GD20::readBytes: Error unlocking thread\n");
//        return (-1);
//    }
    
    //Wire.endTransmission();
    return true;
}


uint8_t Adafruit_L3GD20::SPIxfer(uint8_t x) {
  uint8_t value = 0;
    printf("Adafruit_L3GD20::SPIxfer: SPI mode not enabled\n");

//  for (int i=7; i>=0; i--) {
//    digitalWrite(_clk, LOW);
//    if (x & (1<<i)) {
//      digitalWrite(_mosi, HIGH);
//    } else {
//      digitalWrite(_mosi, LOW);
//      }
//    digitalWrite(_clk, HIGH);
//    if (digitalRead(_miso))
//      value |= (1<<i);
//  }

  return value;
}
