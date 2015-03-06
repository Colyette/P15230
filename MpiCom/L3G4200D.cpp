//Arduino 1.0+ only
//bildr.org under MIT licensing
/**
 * \brief modifications made to use on Raspberry Pi with Linux userspace
 * drivers
 * \modified Alyssa Colyette
 */


//#include <Wire.h>
#include <stdio.h>
#include <linux/i2c-dev.h>  //linux userspace I2C driver
#include <sys/ioctl.h>      //opens posix compliant drivers
#include <errno.h>          //reading error codes delivered by bad io
#include <fcntl.h>          //read, write file handler
#include <unistd.h>         //constants for posix compliance
#include "L3G4200D.h"



//void setup(){
//
//  Wire.begin();
//  Serial.begin(9600);
//
//  Serial.println("starting up L3G4200D");
//  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
//
//  delay(1500); //wait for the sensor to be ready 
//}

L3G4200D::L3G4200D(int e_dev_handle) {
    dev_handle = e_dev_handle;
    setupL3G4200D(2000);
    ::sleep(1); //wait for sensor to be ready
}


//void loop(){
//   getGyroValues();  // This will update x, y, and z with new values
//
//  Serial.print("X:");
//  Serial.print(x);
//
//  Serial.print(" Y:");
//  Serial.print(y);
//
//  Serial.print(" Z:");
//  Serial.println(z);
//
//  delay(100); //Just here to slow down the serial to make it more readable
//}

/**
 * \brief the values are stored as private variables at the moment
 * //TODO getters or return array
 */
void L3G4200D::getGyroValues(){
    uint8_t xMSB,xLSB, yMSB, yLSB, zMSB, zLSB;
    xMSB = readRegister(L3G4200D_Address, 0x29);
    xLSB = readRegister(L3G4200D_Address, 0x28);
    x = ((xMSB << 8) | xLSB);

    yMSB = readRegister(L3G4200D_Address, 0x2B);
    yLSB = readRegister(L3G4200D_Address, 0x2A);
    y = ((yMSB << 8) | yLSB);

    zMSB = readRegister(L3G4200D_Address, 0x2D);
    zLSB = readRegister(L3G4200D_Address, 0x2C);
    z = ((zMSB << 8) | zLSB);
}




int L3G4200D::setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

/**
 * \breif writes one byte to a provided reg address
 * TODO -- make all read and write Registers generic!
 */
int L3G4200D::writeRegister(int deviceAddress, uint8_t address, uint8_t val) {
//    Wire.beginTransmission(deviceAddress); // start transmission to device
    if( ioctl( dev_handle, I2C_SLAVE, GYRO_ADDR) < 0 ){
        err = errno ;
        printf( "L3G4200D::writeRegister: I2C bus cannot point to gyro comp of GY-80 Slave: errno %d\n",err);
        return 0;
    }
//    Wire.write(address);       // send register address
    if ( write(dev_handle,&address, 1 ) != 1 ){
        err = errno ;
        printf("L3G4200D::writeRegister: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.write(val);         // send value to write
    if ( write(dev_handle,&val, 1 ) != 1 ){
        err = errno ;
        printf("L3G4200D::writeRegister: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.endTransmission();     // end transmission
}

/**
 * \brief reads one byte from provided  (reg) address,
 *  \returns negative number if failure... should fix for read byte might be negative...
 */
int L3G4200D::readRegister(int deviceAddress, uint8_t address){

    int v;
    int err,rec;
//    Wire.beginTransmission(deviceAddress);
    if( ioctl( dev_handle, I2C_SLAVE, GYRO_ADDR) < 0 ){
        err = errno ;
        printf( "L3G4200D::readRegister: I2C bus cannot point to gyro comp of GY-80 Slave: errno %d\n",err);
        return 0;
    }
//    Wire.write(address); // register to read
    if ( write(dev_handle,&address, 1 ) != 1 ){
        err = errno ;
        printf("L3G4200D::readRegister: change write register address: errno %d\n",err);
        return -1;
    }
    
//    Wire.endTransmission();

//    Wire.requestFrom(deviceAddress, 1); // read a byte
//    while(!Wire.available()) {
//        // waiting
//    }
//
//    v = Wire.read();
    if ((rec = ::read( dev_handle,&v, 1 )) != 1  ) { //TODO read a byte, store in int -> uint8_t
        err = errno ;
        printf("L3G4200D::readRegister: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return -2;
    }
    
    
    return v;
}
