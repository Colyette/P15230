/*
 *
 * @file masterI2Ccom.h
 * @brief  reads i2c packets from slave Arduino devices and IMU sensor
 * @author Alyssa Colyette
 *
 */
#ifndef MASTER_I2C_COM
#define MASTER_I2C_COM

//headers
#include <string>
#include <stdint.h>
#include <pthread.h>    //for threading
#include <mutex>        //for recursive mutex
#include <queue>        //for vector of sub targets
#include "sharedi2cCom.h"
#include "IMU.h"
#include "Grid.h"

//#define M_PI 3.141592653589793238462643 //stolen from Arduino's math.h lib

//for Jetson Course, in meters
#define COURSE_X_DIM (85)
#define COURSE_Y_DIM (85)
#define RESOLUTION   (1) //spaceing between each node/point in meters
#define NUM_SUB_TARGETS (4) // number of stops to make this trip, hardcoded to 4
//hardcoded 4 targets
#define T1_X    (10)
#define T1_Y    (10)
#define T2_X    (10)
#define T2_Y    (20)
#define T3_X    (20)
#define T3_Y    (10)
#define T4_X    (20)
#define T4_Y    (20)

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
	int requestSonar(SonarReqPkt * rPkt);
    
    /**
     * \brief continously reads baro values
     */
    void continousBaroReading();
    
    /**
     * \brief starts the baro thread
     */
    int startBaroThread();
    
    /**
     * \brief stops the baro thread
     */
    void stopBaroThread();
    
    //TEsting functs
    void reqAndprintAccelerameterData();
    void reqAndprintCompassData();
    void reqAndprintBarometerData();
    void reqAndprintGyrometerData();
	
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
     * \brief goes forward the set amount of meters
     * \param meters- the number of meters to go forward
     */
    int forward(double meters);
    
    /**
     * \brief keeps the craft hovering relativly in place
     */
    int hover();

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
    
    //some variable
    int runBaro; //variable
    
    //thread for running the continous baro readings
    pthread_t baroThread;
    
    //mutex for device hangle
    static std::recursive_mutex dev_handle_mutex;
    
    //grid with path planning
    Grid* map;
    //list of sub targets
    std::vector<Node*> subTargetList;
    // saving assumed position in map
    double cur_x_pos;
    double cur_y_pos;

};
#endif //MASTER_I2C_COM
