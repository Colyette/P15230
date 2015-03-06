/*
 *
 * @file masterI2Ccom.h
 * @brief  reads i2c packets from slave Arduino devices
 * @author Alyssa Colyette
 *
 */
#ifndef MASTER_I2C_COM
#define MASTER_I2C_COM

//headers
#include <string>
#include <stdint.h>
#include "sharedi2cCom.h"

//#define M_PI 3.141592653589793238462643 //stolen from Arduino's math.h lib

/**
 * Class for i2c communications with Arudino slaves (PPM signal propeller control outbound and  
 * sonar sensor readings inbound)
 */
class MasterI2Ccom {
public:
	/**
	 * Constructor
	 * 
	 */
	MasterI2Ccom();

	/**
         * Destructor
         * 
         */
	~MasterI2Ccom();
	
	/**
	 * \brief Open the i2c bus on the master pi
	 */
	int openi2cBus();
	
	/** 
	 * \brief Closes the i2c bus on the master pi 
	 */
	int closei2cBus();

	/**
	 * \brief requests a sonar packet from sonar slave Arduino
	 */
	int requestSonar();
	
	/**
	 * \brief requests inertial movement data from the gy-80 sensor
   	 * over I2C
  	 */
	int requestIMU();

	/**
	 * \brief gives 'absolute coordinates to the LIDAR unit
	 * receives distances of approaching objects
	 */
	int updateLIDAR();


	/**
	 * @brief sends a packet for PPM interpretation in the quad motion control
	 * Arduino
 	 * \param: cmd the flight command to send
	 * \param: the parameter to pass, typically a distance
	 */
	int sendPPM(ReqPkt * rPkt);
    
    /**
     * \brief returns the dev handle for i2c, could be uninitialized
     */
    int get_dev_handle(){return dev_handle;}
private:
	
	/**
	 * device handle for the i2c bus
	 */
	int dev_handle;

	/**
	 * \brief Accelerameter address on GY-80 IMU
 	 */
    int accel_addr;//=0x53; //TODO now in ADXL345, and can't be defined in class


	/**
     * \brief Compass address on GY-80 IMU
     */
    int comp_addr;//=0x1E;

	/**
     * \brief Gyroscope address on GY-80 IMU
     */
    int gyro_addr;//=0x69;


	/**
     * \brief Barometer address on GY-80 IMU
     */
    int baro_addr;//= 0x77;



};
#endif //MASTER_I2C_COM
