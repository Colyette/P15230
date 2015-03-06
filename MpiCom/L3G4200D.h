/*******
 * \breif Class for interfacing with the L3G4200D 3-axis gyroscope on the 
 *  GY-80 IMU module
 * \author Alyssa Colyette
 */

#ifndef L3G4200D_H
#define L3G4200D_H

#include <stdint.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define GYRO_ADDR (105)



class L3G4200D {
    
private:
    //int L3G4200D_Address = 105; //I2C address of the L3G4200D
    int x;
    int y;
    int z;
    
    int dev_handle; //opened dev_handler
    
    L3G4200D(); //can't construct without
public:
    
    L3G4200D(int e_dev_handle);
    
    update_dev_handle(int e_dev_handle);
    
    int setupL3G4200D(int scale);
    
    void getGyroValues();
    
    int writeRegister(int deviceAddress, uint8_t address, byte val);
    
    int readRegister(int deviceAddress, byte address);
    
}
#endif //L3G4200D_H
