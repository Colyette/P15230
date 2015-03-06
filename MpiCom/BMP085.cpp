/*Based largely on code by  Jim Lindblom
 
 Get pressure, altitude, and temperature from the BMP085.
 Serial.print it out at 9600 baud to serial monitor.
 */

/**
 * \file BMP085.cpp
 * \brief Modified Arduino code to use on Raspberry Pi with Linux I2C userspace
 *      driver
 *  modified by Alyssa Colyette on 3/5/15.
 *
 */

#include "BMP085.h"
//Arduino 1.0+ Only
//Arduino 1.0+ Only

//#include <Wire.h>


BMP085::BMP085(int e_dev_handle){
    dev_handle = e_dev_handle;
}

int BMP085::update_dev_handle (int n_dev_handle){
    dev_handle = n_dev_handle;
    return 1;
}

int BMP085::setup(){
//    Serial.begin(9600);
//    Wire.begin();
    int ret;
    ret= bmp085Calibration();
    return ret;
}

//TODO some sort of data collect or main
void loop()
{
    float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
    float pressure = bmp085GetPressure(bmp085ReadUP());
    float atm = pressure / 101325; // "standard atmosphere"
    float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters
    
    Serial.print("Temperature: ");
    Serial.print(temperature, 2); //display 2 decimal places
    Serial.println("deg C");
    
    Serial.print("Pressure: ");
    Serial.print(pressure, 0); //whole number only.
    Serial.println(" Pa");
    
    Serial.print("Standard Atmosphere: ");
    Serial.println(atm, 4); //display 4 decimal places
    
    Serial.print("Altitude: ");
    Serial.print(altitude, 2); //display 2 decimal places
    Serial.println(" M");
    
    Serial.println();//line break
    
    delay(1000); //wait a second and get values again.
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
int BMP085::bmp085Calibration()
{
    int ret;
    ac1 = bmp085ReadInt(0xAA);
    ac2 = bmp085ReadInt(0xAC);
    ac3 = bmp085ReadInt(0xAE);
    ac4 = bmp085ReadInt(0xB0);
    ac5 = bmp085ReadInt(0xB2);
    ac6 = bmp085ReadInt(0xB4);
    b1 = bmp085ReadInt(0xB6);
    b2 = bmp085ReadInt(0xB8);
    mb = bmp085ReadInt(0xBA);
    mc = bmp085ReadInt(0xBC);
    md = bmp085ReadInt(0xBE);
    if (ret) { //TODO find error values
        return;
    } else {
        return 0;//TODO ok for now.....
    }
}

// Calculate temperature in deg C
float BMP085::bmp085GetTemperature(unsigned int ut){
    long x1, x2;
    
    x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
    x2 = ((long)mc << 11)/(x1 + md);
    b5 = x1 + x2;
    
    float temp = ((b5 + 8)>>4);
    temp = temp /10;
    
    return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long BMP085::bmp085GetPressure(unsigned long up){
    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;
    
    b6 = b5 - 4000;
    // Calculate B3
    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
    
    // Calculate B4
    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
    
    b7 = ((unsigned long)(up - b3) * (50000>>OSS));
    if (b7 < 0x80000000)
        p = (b7<<1)/b4;
    else
        p = (b7/b4)<<1;
    
    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;
    
    long temp = p;
    return temp;
}

// Read 1 byte from the BMP085 at 'address'
uint8_t BMP085::bmp085Read(uint8_t address)
{
    uint8_t data;
    int err,rec;
    
//    Wire.beginTransmission(BMP085_ADDRESS);
    if( ioctl( dev_handle, I2C_SLAVE, BMP085_ADDRESS) < 0 ){
        err = errno ;
        printf( "BMP085::bmp085Read: I2C bus cannot point to baro comp of GY-80 Slave: errno %d\n",err);
        return 0; //TODO change?
    }
//    Wire.write(address);
    if ( write(dev_handle,&address, 1 ) != 1 ){
        err = errno ;
        printf("BMP085::bmp085Read: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.endTransmission();
    
//    Wire.requestFrom(BMP085_ADDRESS, 1);
//    while(!Wire.available())
//        ;
    if ((rec = ::read( dev_handle,&data, 1 )) != 1  ) { //1 byte
        err = errno ;
        printf("BMP085::bmp085Read: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return -1;
    }
    
//    return Wire.read();
    return data;
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
uint16_t BMP085::bmp085ReadInt(uint8_t address)
{
//    unsigned char msb, lsb;
    uint8_t buffer[2]; //for both read bytes
//    Wire.beginTransmission(BMP085_ADDRESS);
    if( ioctl( dev_handle, I2C_SLAVE, BMP085_ADDRESS) < 0 ){
        err = errno ;
        printf( "BMP085::bmp085ReadInt: I2C bus cannot point to baro comp of GY-80 Slave: errno %d\n",err);
        return 0; //TODO change?
    }

//    Wire.write(address);
    if ( write(dev_handle,&address, 1 ) != 1 ){
        err = errno ;
        printf("BMP085::bmp085ReadInt: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.endTransmission();
//    
//    Wire.requestFrom(BMP085_ADDRESS, 2);
//    while(Wire.available()<2)
//        ;
//    msb = Wire.read();
//    lsb = Wire.read();
//    ;
    if ((rec = ::read( dev_handle,buffer, 2 )) != 2  ) { // reads 2 bytes
        err = errno ;
        printf("BMP085::bmp085ReadInt: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return -1;
    }
    
    return ((uint16_t)buffer[0]<<8) | (uint16_t) buffer[1] ; //combine into 16 bit, assuming msb first
    
//    return (int) msb<<8 | lsb;
}

//WARNING 5 sec delay for conversion request to finish, required for b5 constant....
// Read the uncompensated temperature value
uint16_t BMP085::bmp085ReadUT(){
    uint16_t ut;
    int err,rec;
    
    // Write 0x2E into Register 0xF4
    // This requests a temperature reading
//    Wire.beginTransmission(BMP085_ADDRESS);
    if( ioctl( dev_handle, I2C_SLAVE, BMP085_ADDRESS) < 0 ){
        err = errno ;
        printf( "BMP085::bmp085ReadUT I2C bus cannot point to baro comp of GY-80 Slave: errno %d\n",err);
        return 0; //TODO change?
    }
//    Wire.write(0xF4);
    if ( write(dev_handle,0xF4, 1 ) != 1 ){ //immediate might not work....
        err = errno ;
        printf("BMP085::bmp085ReadUT change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.write(0x2E);
    if ( write(dev_handle,0x2E, 1 ) != 1 ){ //immediate might not work....
        err = errno ;
        printf("BMP085::bmp085ReadUT change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.endTransmission();
    
    // Wait at least 4.5ms
    ::sleep(5);
    
    // Read two bytes from registers 0xF6 and 0xF7
    ut = bmp085ReadInt(0xF6);
    return ut;
}

// Read the uncompensated pressure value
//unsigned long bmp085ReadUP(){
uint32_t BMP085::bmp085ReadUP(){
    int err,rec;
    uint8_t msb, lsb, xlsb;
    uint32_t up = 0;
    
    // Write 0x34+(OSS<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
//    Wire.beginTransmission(BMP085_ADDRESS);
    if( ioctl( dev_handle, I2C_SLAVE, BMP085_ADDRESS) < 0 ){
        err = errno ;
        printf( "BMP085::bmp085ReadUP: I2C bus cannot point to baro comp of GY-80 Slave: errno %d\n",err);
        return 0; //TODO change?
    }
//    Wire.write(0xF4);
    if ( write(dev_handle,0xF4, 1 ) != 1 ){ //immediate might not work....
        err = errno ;
        printf("BMP085::bmp085ReadUP: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.write(0x34 + (OSS<<6));
    if ( write(dev_handle,(0x34 + (OSS<<6) ), 1 ) != 1 ){ //immediate might not work....
        err = errno ;
        printf("BMP085::bmp085ReadUP: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.endTransmission();
    
    // Wait for conversion, delay time dependent on OSS
    //delay(2 + (3<<OSS));
    ::sleep(2 + (3<<OSS) );
    
    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    msb = bmp085Read(0xF6);
    lsb = bmp085Read(0xF7);
    xlsb = bmp085Read(0xF8);
    
    up = (((uint32_t) msb << 16) | ((uint32_t) lsb << 8) | (uint32_t) xlsb) >> (8-OSS);
    
    return up;
}

int BMP085::writeRegister(int deviceAddress, uint8_t address, uint8_t val) {
    int err,rec;
//    Wire.beginTransmission(deviceAddress); // start transmission to device
    if( ioctl( dev_handle, I2C_SLAVE, BMP085_ADDRESS) < 0 ){
        err = errno ;
        printf( "BMP085::writeRegister: I2C bus cannot point to baro comp of GY-80 Slave: errno %d\n",err);
        return 0; //TODO change?
    }
//    Wire.write(address);       // send register address
    if ( write(dev_handle,&address, 1 ) != 1 ){
        err = errno ;
        printf("BMP085::writeRegister: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.write(val);         // send value to write
    if ( write(dev_handle,&val, 1 ) != 1 ){
        err = errno ;
        printf("BMP085::writeRegister: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.endTransmission();     // end transmission
    return 0; //everything is A-Oh-kay
}

//reads a byte
uint8_t BMP085::readRegister(int deviceAddress, uint8_t address){
    int err,rec;
    uint8_t v;
//    Wire.beginTransmission(deviceAddress);
    if( ioctl( dev_handle, I2C_SLAVE, BMP085_ADDRESS) < 0 ){
        err = errno ;
        printf( "BMP085::readRegister: I2C bus cannot point to baro comp of GY-80 Slave: errno %d\n",err);
        return 0; //TODO change?
    }
//    Wire.write(address); // register to read
    if ( write(dev_handle,&address, 1 ) != 1 ){
        err = errno ;
        printf("BMP085::readRegister: change write register address: errno %d\n",err);
        return -1;
    }
//    Wire.endTransmission();
    
//    Wire.requestFrom(deviceAddress, 1); // read a byte
//    while(!Wire.available()) {
//        // waiting
//    }
//    v = Wire.read();

    if ((rec = ::read( dev_handle,&v, 1 )) != 1  ) { //1 byte
        err = errno ;
        printf("BMP085::readRegister: Couldn't read from register: errno %d rec: %d\n",err,rec);
        return -1;
    }

    return v;
}

float BMP085::calcAltitude(float pressure){
    
    float A = pressure/101325;
    float B = 1/5.25588;
    float C = pow(A,B);
    C = 1 - C;
    C = C /0.0000225577;
    
    return C;
}