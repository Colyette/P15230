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

#define COMSTATUS_TEENSY (0x4)    //flag set mask for teensy communications status
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
     * \brief from the provided next point, will rotate and fly forward to
     * that point
     * \param navPoint the targeted node to move to
     */
    int navToPoint(Node* navPoint);
    
    /**
     * \brief update cur_pos node ptr
     */
    int set_cur_pos(Node* n_cur_pos){ cur_pos = n_cur_pos;}

	/**
	 * \brief gives 'absolute coordinates to the LIDAR unit
	 * receives distances of approaching objects
	 */
	int updateLIDAR();

    
    /**
     * \brief calls for LIDAR and SONAR sensor reading to update map
     */
    int updateMap();

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
    
    /**
     *\breif returns the Node that is set as the indexed target
     * \param targetNum - the indexed target 
     * \return Node* - the node set as the targeted location at that index
     */
    Node* getSubtarget(int targetNum){return subTargetList.at(targetNum);}
    
    /**
     * \brief returns the next node to go to from current discret location
     */
    Node* getNextNavPoint( Node* finish);
    
    /**
     * \brief getter for i2c slave communications status flg
     */
    uint8_t geti2cComStatus(){return i2cComStatus;}
    
    //used for the craft to know what is the current orientation for object
    //detection map update
    enum static_orient {UPH,DOWNH,LEFTH,RIGHTH} static_orient_t;
    
private:
	
    static_orient xHeading;
    
	/**
	 * device handle for the i2c bus
	 */
	int dev_handle;
    
    uint8_t i2cComStatus; //tells the status of i2c slaves
    
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
    Grid* gridMap;
    //list of sub targets
    std::vector<Node*> subTargetList;
    // saving assumed position in map
    double cur_x_pos;   //CURRENTLY NOT USED
    double cur_y_pos;   //CURRENTLY NOT USED
    Node* cur_pos;
    
    /**
     * \brief using the current position, figures rotaion positions for
     * node navigation
     */
    int _calibrateOrientaion();
    
    double upHeading;
    double downHeading;
    double leftHeading;
    double rightHeading;

};
#endif //MASTER_I2C_COM
