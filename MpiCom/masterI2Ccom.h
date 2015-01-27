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
	 * @brief Open the i2c bus on the master pi
	 */
	int openi2cBus();
	
	/** 
	 * @brief Closes the i2c bus on the master pi 
	 */
	int closei2cBus();

	/**
	 * @brief requests a sonar packet from sonar slave Arduino
	 */
	int requestSonar();

	/**
	 * @brief sends a packet for PPM interpretation in the quad motion control
	 * Arduino
	 */
	int sendPPM();
private:
	
	/**
	 * device handle for the i2c bus
	 */
	int dev_handle;

};
#endif //MASTER_I2C_COM
