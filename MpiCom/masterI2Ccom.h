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
#include "IMU.h"

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
    
    //TEsting functs
    void reqAndprintAccelerameterData();
    void reqAndprintCompassData();
	
	/**
     * \brief lifts the craft from initial takeoff
     */
    int launch( double height, double wiggle);
    
    /**
     *  \brief lands the craft from flying position
     */
    int land();
    
    /**
     * \brief rotates the copter left/counter-clockwise to the given degree
     */
    int rotate(double deg );

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
    
    IMU* imu; //imu sensor for determining current pos to set pos

    //soley the direction the copter is facing currently
    double current_orientation;

};
#endif //MASTER_I2C_COM
