/*
 *
 * @file masterI2Ccom.cpp
 * @brief  reads i2c packets from slave Arduino devices and IMU sensor
 * @author Alyssa Colyette
 *
 */

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <math.h>
#include "masterI2Ccom.h" 
#include "sharedi2cCom.h"
//#include "ADXL345.h"    //accelerameter lib

#include "jmPID.h" //may be compostie of class

//inilize static mutex for I2C driver sharing
std::recursive_mutex MasterI2Ccom::dev_handle_mutex;

//#################################BEGIN For Running the Barometer reading thread
#define NUM_SAMPLES (10) //number of samples to take before selecting median
//#define AVERAGE

/**
 * \brief serves as a helper function for multi-threading within the same class instance
 */
void * BarometerHelper(void* instance) {
    MasterI2Ccom* c_instance = (MasterI2Ccom*) instance;
    printf("BarometerThread: Going into continous reading function\n");
    c_instance->continousBaroReading(); //casted to instantiated class and run main thread funct
    return 0 ;
}



/**
 * \brief continously reads barometer data, and selects the candidate reading
 * from every 10 samples
 */
void MasterI2Ccom::continousBaroReading(){
    float lastValues[NUM_SAMPLES];
    float ravg;
    int count=0;
    int i;
    bool swapped; // for sorting
    int n = NUM_SAMPLES; //for values to go through in to be sorted
    float temp; //temp for swapping in sortings
    
    while(runBaro){
        imu -> getAltitude(); //Updates class variables
        lastValues[count] = (imu->alt_m-imu->c_base_alt);
        //printf("[%d]got %fm\n",count,lastValues[count]);
        count++;
        if(count == NUM_SAMPLES ){
#ifdef  AVERAGE
          //  printf("calc avg\n");
            ravg = 0;
            for (i =0; i<NUM_SAMPLES;i++) {
                ravg += lastValues[i];
            }
            imu->avgBaro = ravg/NUM_SAMPLES;
            //printf("Avg:%f meter diff\n",imu->avgBaro);
#else
            //using median filtering
            //order using bubble sort to sort
            do {
                swapped = false;
                for(i = 1; i<=n-1; i++){
                    if (lastValues[i-1] > lastValues[i]) { //then swap
                        temp = lastValues[i];
                        lastValues[i] = lastValues[i-1];
                        lastValues[i-1] = temp;
                        swapped = true;
                    }// else we hadn't swapped this round
                }
                n= n-1;
            } while (!swapped);
            //get median value
            imu->avgBaro = lastValues[(int)ceil(NUM_SAMPLES/2.0)];
            //printf("Median:%f meter diff\n",imu->avgBaro); would be bad for mult threading
#endif
            count =0;
        } //end of collected samples for filtering
    }
}

/**
* \brief starts the baro thread
*/
int MasterI2Ccom::startBaroThread(){
    runBaro=1;
    if(pthread_create(&baroThread, NULL,BarometerHelper,this)){
        printf("MainThread: error creating BaroThread thread\n");
        return -1;
    }
    return 1;
}

void MasterI2Ccom::stopBaroThread() {
    runBaro =0;
    pthread_join(baroThread,NULL);
    printf("BaroThread closed\n");
    return;
}
//#################################END For Running the Barometer reading thread

//#################################BEGIN Constructor and Destructor
MasterI2Ccom:: MasterI2Ccom(){
	printf("MasterI2Ccom:: Class initialized \n");
	dev_handle = -1;
    //dev_handle_mutex = PTHREAD_MUTEX_INITIALIZER;
    //pthread_mutex_init(&dev_handle_mutex,NULL);
    
    //init map with course (navigatable) dimensions
    gridMap = new Grid (COURSE_X_DIM,COURSE_Y_DIM,RESOLUTION);
    gridMap->initializeGrid();
    gridMap->add_neighbors();
    subTargetList.push_back(gridMap->getNode(T1_X,T1_Y));
    subTargetList.push_back(gridMap->getNode(T2_X,T2_Y));
    subTargetList.push_back(gridMap->getNode(T3_X,T3_Y));
    subTargetList.push_back(gridMap->getNode(T4_X,T4_Y));
    printf("MasterI2cCom:: gridMap initialized with referenced subtargetlist \n");
    cur_pos = gridMap-> getNode(0,0); //TODO remove
//    cur_x_pos = 2;
//    cur_y_pos = 2;
    
}

MasterI2Ccom::~MasterI2Ccom() {
	printf("~MasterI2Ccom:: Class destroyed\n");
}

/**
 * \breif using the current position, figures rotaion positions for
 * node navigation
 */
int MasterI2Ccom::_calibrateOrientaion(){
    //TODO TEST
    if(!imu -> getCompassValues()){
        return 0;
    }
    upHeading = imu->heading;       //assume facing postion is now up
    downHeading = upHeading -180;
    if (downHeading < 0) {
        downHeading += 360; //re-adjust for 0-360
    }
    leftHeading = upHeading -90;
    if (leftHeading < 0) {
        leftHeading += 360; //re-adjust for 0-360
    }
    rightHeading = upHeading +90;
    if (rightHeading > 360) {
        rightHeading -= 360; //re-adjust for 0-360
    }
    printf ("MasterI2Ccom::_calibrateOrientaion(): Calibrated Heading as UP:%fdeg DOWN:%fdeg LEFT:%fdeg RIGHT:%fdeg\n",
            upHeading,downHeading,leftHeading,rightHeading );
    return 1; //TODO if IMU reading unsuccessful...
}
//#################################END Constructor and Destructor

//#################################BEGIN I2C Communications Functions
/**
 * \brief Open the I2C bus on the master Pi, then initilizes the IMU
 * if sucessful
 */
int MasterI2Ccom::openi2cBus() {
	if(dev_handle > 0) {
		printf("openi2cBus:: I2C bus already opened\n");
		return 0;
	}else{
		if( ( dev_handle = ::open("/dev/i2c-1", O_RDWR) ) < 0 ){
			printf("openi2cBus:: Couldn't open I2C bus\n") ;
			
			// initial open failed, don't bump the ref count
			return -1 ;
        } else{ //Pass handler to IMU
            printf("passing desc=%d to IMU\n",dev_handle);
            imu =  new IMU(dev_handle,&dev_handle_mutex);
            //check IMU module communications status flg
            i2cComStatus |= imu->getcomStatus();
            printf("MasterI2Ccom::openi2cBus():IMU init, calibrating orientation\n");
            if (!_calibrateOrientaion()) { //
                printf("MasterI2Ccom::openi2cBus(): couldn't calibrate orientation\n");
                //TODO handle
                //update enum to UP
                xHeading= UPH;
                
            }
        }
	}
	printf("openi2cBus:: Bus opened desc=%d\n",dev_handle);
 	return 1;
}

int MasterI2Ccom::closei2cBus() {
	//TODO close linux driver
	int ret;
    free(imu); //remove IMU instance
	if ( dev_handle < 0){ //valid handlers can be 0...
		printf("closei2cBus:: There isn't a valid device open\n");
		return 0;
	} else {
		ret = ::close( dev_handle ) ;
 		if( ret ){
			printf( "closei2cBus:: Couldn't close I2C bus\n");
			return -1;
		}
        
	}
	printf("closei2cBus:: Bus closed\n");
	return 1;
}

/**
 * @brief
 * @precon i2c bus must be open 
 */
int MasterI2Ccom::requestSonar(SonarReqPkt * rPkt){
	uint8_t mbuff[5]; //sizeof packet
	//SonarReqPkt rPkt;
	int err,rec;
	uint8_t dumData = 0xAA;

    uint32_t readData;

	printf("requestSonar:: Requested packet\n");
#ifndef NO_I2C
    if (geti2cComStatus() & COMSTATUS_TEENSY) {
#endif
        std::unique_lock<std::recursive_mutex> lck(dev_handle_mutex);

        //point to sonar slave
        if( ioctl( dev_handle, I2C_SLAVE, ppmArduinoAdd ) < 0 ){
            err = errno ;
            printf( "requestSonar:: I2C bus cannot point to Sonar Slave: errno %d \n",err);
            return 0;
        }
        //TODO modify for receiving sonar packets
        if (    write(dev_handle, &dumData, 1 ) != 1 ){
            err = errno ;
            printf("requestSonar:: Couldn't send sonar packet: errno %d\n",err);
            return -1;
        }	
        ::usleep(100000); //wait some time ::usleep(50000)
        printf("Size of SonarReqPkt: %d\n",sizeof(SonarReqPkt));
        if ((rec = ::read( dev_handle,rPkt, sizeof(SonarReqPkt) )) != sizeof(SonarReqPkt)  ) { // sized in bytes
            err = errno ;
                    printf("requestSonar:: Couldn't get sonar packet: errno %d rec: %d\n",err,rec);
                    return -1;
        }

        //	printf("buff: 0x%x 0x%x 0x%x 0x%x 0x%x \n", mbuff[0], mbuff[1], mbuff[2], mbuff[3],mbuff[4]);
        //	printf("packet: 0x%d 0x%d 0x%d 0x%d 0x%d \n", rPkt.header, rPkt.sonar1, rPkt.sonar2, rPkt.sonar3,rPkt.sonar4);
        printf ("pkt #:%d\ns1:%dcm\ts2:%dcm\ts3:%dcm\ts4:%dcm\n cur:%fA\tvolt:%fV\n",
                rPkt->header, (int16_t)rPkt->sonar1,(int16_t)rPkt->sonar2,(int16_t)rPkt->sonar3,(int16_t)rPkt->sonar4,
                (double)rPkt->current, (double) rPkt->voltage); //TODO might changed from unsigned

        //translate to packet, mbuff
        //readData = (mbuff[0] <<24) || (mbuff[1] << 16) || (mbuff[2] <<8 ) || mbuff[3];
        //	printf("Read 0x%x \n",readData);
        //printf("bytes: 0x%x%x%x%x \n", mbuff[0], mbuff[1], mbuff[2], mbuff[3]);

        printf("RequestSonar:: Received packet\n");
#ifndef NO_I2C
    } else {
        printf("RequestSonar:: Restart Program I2C Teensy link down\n");
        return -1;
    }
#endif
	return 1;
}

/**
 * \brief sends a packet to the Arduino PPM slave in order relay
 * 	a flight cmd
 * \param cmd - the command to be sent to the Arduino
 */
int MasterI2Ccom::sendPPM(ReqPkt* rPkt){
    //TODO send a pkt to the movement
    //ReqPkt rPkt;
    int err,rec;
#ifndef NO_I2C
    if (geti2cComStatus() & COMSTATUS_TEENSY) {
#endif
        std::unique_lock<std::recursive_mutex> lck(dev_handle_mutex);

        //point to sonar slave
        if( ioctl( dev_handle, I2C_SLAVE, ppmArduinoAdd) < 0 ){
            err = errno ;
            printf( "sendPPM:: I2C bus cannot point to PPM Slave: errno %d \n",err);
            return 0;
        }
        //TODO modify for receiving sonar packets
        if (    write(dev_handle,rPkt, sizeof(ReqPkt) ) != sizeof(ReqPkt) ){
            err = errno ;
            printf("sendPPM:: Couldn't send flight request packet: errno %d\n",err);
            return -1;
        }
        ::usleep(1000);//::usleep(100000); //wait some time ::usleep(50000)
        printf("Size of ReqPkt: %d\n",sizeof(ReqPkt));
        if ((rec = ::read( dev_handle,rPkt, sizeof(ReqPkt) )) != sizeof(ReqPkt)  ) { // sized in bytes
            err = errno ;
            printf("sendPPM:: Couldn't get flight request packet reply: errno %d rec: %d\n",err,rec);
            return -1;
        }

        printf ("header #:%d, throttle:%u, yaw:%u, pitch:%u, roll:%u\n", rPkt->header, rPkt->throttle,
                rPkt->yaw,rPkt->pitch, rPkt->roll);

        printf("sendPPM:: Received packet\n");
#ifndef NO_I2C
    } else {
        printf("sendPPM:: Restart Program I2C Teensy link down\n");
        return -1;
    }
#endif
    return 1;
}



//#################################### triggers i2C imu reads
//TEsting functs
void MasterI2Ccom::reqAndprintAccelerameterData(){
    imu -> getAccelerationValues();
    printf("Accel:\nx:%d\ty:%d\tz:%d\n", imu->a_x, imu->a_y, imu->a_z);
    
}

void MasterI2Ccom::reqAndprintCompassData(){
    imu -> getCompassValues();
    printf("Comp:\nx:%d\ty:%d\tz:%d\n", imu->c_x, imu->c_y, imu->c_z);
    printf("heading:%f\n",imu->heading);
}

void MasterI2Ccom::reqAndprintBarometerData() {
   //done in thread now imu -> getAltitude(); //TODO remove prints in getAltitude
    printf("Pressure = %d Pa\n",imu->temp );
    printf("Altitude = %f meters\n", imu->alt_m);
    printf("Pressure at sealevel (calculated) = %d Pa\n", imu->slPressure );
    printf("Real altitude = %f meters\n",  imu->r_alt_m);
    printf("Altitude diff from calibration %f meters\n", (imu->alt_m-imu->c_base_alt) );
    printf("Avg Altitude diff %f m\n",imu->avgBaro);
}

void MasterI2Ccom::reqAndprintGyrometerData() {
    imu->getGryoValues();
    printf("Gyro values:\nx:%f\ty:%f\tz:%f\n", imu->g_x,imu->g_y, imu->g_z);

    
}

//##########################
//#################################END I2C Communications Functions

/**
 * \brief gives 'absolute coordinates to the LIDAR unit
 * receives distances of approaching objects
 */
int MasterI2Ccom::updateLIDAR(){
    
    return 1;
}

/**
 * \brief calls for LIDAR and SONAR sensor reading to update map
 */
int MasterI2Ccom::updateMap(){
    
    SonarReqPkt sPkt;
    Node * n1;
    Node * n2; //for removing edges
    double alpha;    //offset
    int n_1_x,n_1_y,n_2_x, n_2_y; //for mapping node coordinates
    int updateflg=0;
    int c_x,c_y;
    
    uint16_t sLeft,sRight,sBack,sUp;
    
    if (requestSonar(&sPkt) <1) {
        printf("MasterI2cCom::updateMap() error requesting sensor reading pkt\n");
        return -1;
//        printf("simulating right sonar detect\n");
        //Simulating constant right sensing
//        if (cur_pos ->getYcoord() > 15) {
//            sPkt.sonar2 =0;
//            sPkt.sonar1 =0;
//            sPkt.sonar3 =0;
//        }else {
//            sPkt.sonar2 = 70;
//            sPkt.sonar1 =0;
//            sPkt.sonar3 =0;
//        }
//        printf("Left Sonar:%dcm\n",sPkt.sonar1);
//        printf("Right Sonar:%dcm\n",sPkt.sonar2);
//        printf("Back Sonar:%dcm\n",sPkt.sonar3);
    }
    c_x = cur_pos->getXcoord();
    c_y = cur_pos->getYcoord();
    
    //RE-ORIENT sensors....
    printf("xHeading:%d\n",xHeading);
    switch (xHeading) {
        case UPH:
            sLeft = sPkt.sonar1;
            sRight = sPkt.sonar2;
            sBack = sPkt.sonar3;
            sUp = 0;
            break;
        case DOWNH:
            sLeft = sPkt.sonar2;
            sRight = sPkt.sonar1;
            sBack = 0;
            sUp = sPkt.sonar3;
            break;
        case LEFTH:
            sLeft = 0;
            sRight = sPkt.sonar3;
            sBack = sPkt.sonar1;
            sUp = sPkt.sonar2;
            break;
        case RIGHTH:
            sLeft = sPkt.sonar3;
            sRight = 0;
            sBack = sPkt.sonar2;
            sUp = sPkt.sonar1;
            break;
        default:
            break;
    }
    
    
    //TODO check sonar ranges
    if ((sLeft >0x3) ) { //left of map
        //blklist left detection
        alpha = floor( (((float)sPkt.sonar1)/100)/RESOLUTION   );
        n_1_x = c_x + alpha;
        n_2_x = c_x + (alpha-1);
        n_1_y = c_y;
        n_2_y = c_y;
        if ( (n_1_x >= 0)  && (n_2_x<=COURSE_X_DIM)) { //valid nodes to remove
            n1 = gridMap->getNode(n_1_x,n_1_y);
            n2 = gridMap->getNode(n_2_x,n_2_y);
            
            printf("S1-Left\n");
            if ((n1!=NULL) && (n2!=NULL) ) {
                n1->printNodeCord();
                n2->printNodeCord();
                
                if (gridMap->removeEdge(n1,n2) ){
                    printf("MasterI2Ccom::updateMap: L Removed edge from N(%d,%d) to N(%d,%d)~~~~~~~~\n",n_1_x,n_1_y,n_2_x,n_2_y);
                    updateflg |=0x02;
                }
            }else { //else not valid node
                printf("MasterI2Ccom::updateMap: Invalid edges\n");
            }
            
        }
    }else{
        //not in detection range for sonar, ignore
        printf("ignore Left sonar\n");
    }
    
    
    //TODO check sonar ranges
    if ((sRight >0x3)) { // right of map
//        printf("%f\n",((float)sPkt.sonar2));
        //blklist right detection
        alpha = floor ( ( ((float)sPkt.sonar2)/100)/RESOLUTION);
//        printf("alpha:%f\n",alpha);
        n_1_x = c_x - alpha;
        n_2_x = c_x - (alpha-1);
        n_1_y = c_y;
        n_2_y = c_y;
        if ( (n_2_x <= COURSE_X_DIM) && (n_1_x >=0) ) { //valid nodes to remove
//            printf("getting (%d,%d)\n",n_1_x,n_1_y);
            n1 = gridMap->getNode(n_1_x,n_1_y);
//            printf("getting (%d,%d)\n",n_2_x,n_2_y);
            n2 = gridMap->getNode(n_2_x,n_2_y);
            
            printf("S2-Right\n");
            if ((n1!=NULL) && (n2!=NULL) ) {
                n1->printNodeCord();
                n2->printNodeCord();
                
                if (gridMap->removeEdge(n1,n2) ){
                    printf("MasterI2Ccom::updateMap: R Removed edge from N(%d,%d) to N(%d,%d)~~~~~~~~\n",n_1_x,n_1_y,n_2_x,n_2_y);
                     updateflg |=0x04;
                }
            }else { //else not valid node
                printf("MasterI2Ccom::updateMap: Invalid edges\n");
            }
        }else{
//            printf("some range error in right sonar checking\n");
//            printf("n1(%d,%d) n2(%d,%d)\n");
        }
    }else{
        //not in detection range for sonar, ignore
        printf("ignore Right detection\n");
    }
    
    //TODO check sonar ranges
    if ((sBack > 0x3) ) { //back of map
        //blklist back detection, not really needed?
        alpha = floor (   ( ( (float)sPkt.sonar3)/100)/RESOLUTION);
        n_1_x = c_x;
        n_2_x = c_x;
        n_1_y = c_y + alpha;
        n_2_y = c_y + (alpha-1);
        if ( (n_1_y >= 0) && (n_2_y<= COURSE_Y_DIM)) { //valid nodes to remove
            n1 = gridMap->getNode(n_1_x,n_1_y);
            n2 = gridMap->getNode(n_2_x,n_2_y);
            
            printf("S3-Back\n");
            if ((n1!=NULL) && (n2!=NULL) ) {
                n1->printNodeCord();
                n2->printNodeCord();
                
                if (gridMap->removeEdge(n1,n2) ){
                    printf("MasterI2Ccom::updateMap:B Removed edge from N(%d,%d) to N(%d,%d)~~~~~~~~\n",n_1_x,n_1_y,n_2_x,n_2_y);
                     updateflg |=0x08;
                }
            }else { //else not valid node
                printf("MasterI2Ccom::updateMap: Invalid edges derived\n");
            }
        }
    }else{
    //not in detection range for sonar, ignore
        printf("ignore back detection\n");
    }
    
    //TODO check sonar ranges
    if ((sUp > 0x3) ) { //up of map
        //blklist back detection, not really needed?
        alpha = floor (   ( ( (float)sPkt.sonar3)/100)/RESOLUTION);
        n_1_x = c_x;
        n_2_x = c_x;
        n_1_y = c_y - alpha;
        n_2_y = c_y - (alpha-1);
        if ( (n_1_y >= 0) && (n_2_y<= COURSE_Y_DIM)) { //valid nodes to remove
            n1 = gridMap->getNode(n_1_x,n_1_y);
            n2 = gridMap->getNode(n_2_x,n_2_y);
            
            printf("S3-Back\n");
            if ((n1!=NULL) && (n2!=NULL) ) {
                n1->printNodeCord();
                n2->printNodeCord();
                
                if (gridMap->removeEdge(n1,n2) ){
                    printf("MasterI2Ccom::updateMap: U Removed edge from N(%d,%d) to N(%d,%d)~~~~~~~~\n",n_1_x,n_1_y,n_2_x,n_2_y);
                    updateflg |=0x08;
                }
            }else { //else not valid node
                printf("MasterI2Ccom::updateMap: Invalid edges derived\n");
            }
        }
    }else{
        //not in detection range for sonar, ignore
        printf("ignore up detection\n");
    }

    //do lidar stuff!
    if (updateLIDAR() ) {
        updateflg |= 0x01;
    }
    return updateflg;
}

//############################################BEGIN Some generic flight commands###############################
/**
 * \brief lifts the craft from initial takeoff, could be used to get higher
 * height (use 0.1m increments)
 */
int MasterI2Ccom::launch( double height, double wiggle){
    SonarReqPkt spkt;
    ReqPkt pkt;
    //ensure some defaults
    pkt.header = UP;
    pkt.throttle =0;
    pkt.yaw =0;
    pkt.pitch = 0;
    pkt.roll =0;
    //iffy control
    double estHeight; //estimated height
    double baro_height= imu->avgBaro; //TODO sample height, done in thread
#ifdef HF
    requestSonar(&spkt);
    estHeight = 0.8*spkt.sonar4/100 + 0.2*baro_height; //TODO biasing the sonar input first, need to convert to signed
#else
    estHeight = baro_height;
#endif
    //reqAndprintBarometerData();
    while ( estHeight != height) { //basically while forever
        if( (estHeight>=(height -wiggle)) & (estHeight<=(height+wiggle)) ){ //in an acceptable height range
            printf("AT CORRECT ALTITUDE\n");
            return 1; //in acceptable range
        }
        if (estHeight <= 0.010 ) { //TODO initial lift may need to be stronger, may need sonar first
            pkt.header =UP;
            pkt.throttle= 2000; //some full throt value
            //sendPPM( &pkt );
            printf("UP HARD!\nEst: %f\n",estHeight);
        }
        else if (estHeight < height) { //raise copter
            pkt.header = UP;
            pkt.throttle = 1500; // go up moderately
            pkt.altitudeHold = height*100; // hopefully high enought for altitude hold
            //sendPPM(&pkt);
            printf("UP\nEst: %f\n",estHeight);
            
        }else if (estHeight > height) { //lower copter
            pkt.header= DOWN; //over shoot some, need to release
            pkt.throttle = 1000; //
            //sendPPM(&pkt);
            printf("Down\nEst: %f\n",estHeight);
            
        }
        //TODO sample height again
        baro_height= imu->avgBaro; //TODO sample height, done in thread
#ifdef HF
        requestSonar(&spkt);
        estHeight = 0.8*spkt.sonar4/100 + 0.2*baro_height; //biasing the sonar input first
#else
        estHeight = baro_height;
#endif
        ::sleep(1); // not realiable for real time.. I think (time to actualize held command)
        //reqAndprintBarometerData();
    }
    
}


/**
 *  \brief lands the craft from flying position
 */
int MasterI2Ccom::land(){
    ReqPkt pkt;
    SonarReqPkt spkt;
    //ensure some defaults
    pkt.header = DOWN;
    pkt.throttle =0;
    pkt.yaw =0;
    pkt.pitch = 0;
    pkt.roll =0;
    double estHeight; //estimated height
    double baro_height= imu->avgBaro; //TODO sample height, done in thread
#ifdef HF
    requestSonar(&spkt);
    estHeight = 0.8*spkt.sonar4/100 + 0.2*baro_height; //biasing the sonar input first
#else
    estHeight = baro_height;
#endif

    //iffy control
    while( estHeight >=0.1) { //while not in some safe landing distance
        //send low throttle cmds to flight controller
        pkt.header = DOWN;
        pkt.throttle =1000; //some low values
        ///sendPPM(&pkt);
        
        //TODO sample height again
        baro_height= imu->avgBaro; //TODO sample height, done in thread
#ifdef HF
        requestSonar(&spkt);
        estHeight = 0.8*spkt.sonar4/100 + 0.2*baro_height; //biasing the sonar input first
#else
        estHeight = baro_height;
#endif
        printf("%fm\n", estHeight);
        
        //TODO what if something is located below (should be in object detection framework) sonar reads weird
        ::sleep(1); // not realiable for real time.. I think (time to actualize held command)
    }
    printf("Landed\n");
    //TODO stop motors
    //TODO test wiggle, send ppm for att hold
    pkt.header = STOP;
    pkt.throttle = 0; //stop throttle completely
    ///sendPPM(&pkt);

    return 1;
    
}

/**
 * \brief rotates the copter to the given degree (0-360)
 */
int MasterI2Ccom::rotate(double deg ){
    ReqPkt pkt;
    double rChoice,lChoice; //turning effort
    double cur_pos;
    double wiggleR,wiggleL;
    //ensure some defaults
    pkt.header = DOWN;
    pkt.throttle =0; //todo, for alt hold
    pkt.yaw =0;
    pkt.pitch = 0;
    pkt.roll =0;
    //suppose cur =0 then,
    float s_SP;
    wiggleR = deg + 2.5;
    wiggleL = deg - 2.5;
    do{
    imu -> getCompassValues();
        cur_pos = imu->heading;
        s_SP = deg - cur_pos; //current offset
        //s_SP = cur_pos-deg;
    if ( (wiggleR >= cur_pos) && (wiggleL <= cur_pos)  ) { //good enough [+/- 2.5deg var]{
        //TODO test wiggle, send ppm for att hold
        pkt.header = STOP;
        pkt.yaw = 1000; //stop rotation
        //sendPPM(&pkt);
        printf("THERE\n");
        return 1;
     } else if (s_SP >=0) {
         if (s_SP >180) {
             //got left
             pkt.header = LEFT;
             pkt.yaw = 0; // going cc
             //sendPPM(&pkt);
             printf("GO LEFT\n");
             printf("s_SP:%f\n",s_SP);
             printf("cur:%f\n",cur_pos);
         }else {
            //go right
             pkt.header = RIGHT;
             pkt.yaw = 2000;
             //sendPPM(&pkt);
             printf("GO RIGHT\n");
             printf("s_SP:%f\n",s_SP);
             printf("cur:%f\n",cur_pos);
         }
    }else {
        if (s_SP < -180) {
            //go right
            pkt.header = RIGHT;
            pkt.yaw = 2000;
            //sendPPM(&pkt);
            printf("GO RIGHT\n");
            printf("s_SP:%f\n",s_SP);
            printf("cur:%f\n",cur_pos);
        } else {
        //go left
            pkt.header = LEFT;
            pkt.yaw = 0; // going cc
            //sendPPM(&pkt);
            printf("GO LEFT\n");
            printf("s_SP:%f\n",s_SP);
            printf("cur:%f\n",cur_pos);
        }
    }
    ::sleep(1); // not realiable for real time.. I think (time to actualize held command)
    }while (cur_pos != s_SP);
    
    //Shouldn't get here...
    return 1;
}

//TODO ##### should reverse biasing be in the movement command or hover?????
/**
 * \brief goes forward the set amount of meters
 * \param meters- the number of meters to go forward
 */
int MasterI2Ccom::forward(double meters){
    //TODO
    printf("TODO\n");
    //::sleep(1);
}

/**
 * \brief keeps the craft hovering relativly in place
 */
int MasterI2Ccom::hover(){
    //TODO
    printf("TODO\n");
}

//###################################END some generic flight commands
/**
 * \brief from the provided next point, will rotate and fly forward to
 * that point
 * \param navPoint the targeted node to move to
 */
int MasterI2Ccom::navToPoint(Node* navPoint){
    //TODO MAYBE CHECK FOR OBJECT UPDATES HERE...
    if (navPoint == cur_pos->getNeighbor(UP)) { //up neighbor is next
        printf("Turning Forward\n");
        //rotate up, but are we up?
        rotate(upHeading);
        //set cur heading-ish enum
        xHeading=UPH;
        //go forward
        printf("Going forward %dm\n",RESOLUTION);
        forward(RESOLUTION);
        printf("hover\n");
        hover();
    }
    else if (navPoint == cur_pos->getNeighbor(DOWN)){
        printf("Turning Down\n");
        //rotate back
        rotate(downHeading);
        //set cur heading-ish enum
        xHeading=DOWNH;
        //go forward
        printf("Going forward %dm\n",RESOLUTION);
        forward(RESOLUTION);
        printf("hover\n");
        hover();
    }
    else if (navPoint == cur_pos->getNeighbor(LEFT)) {
        printf("Turning left\n");
        //rotate left
        rotate(leftHeading);
        //set cur heading-ish enum
        xHeading=LEFTH;
        //go forward
         printf("Going forward %dm\n",RESOLUTION);
        forward(RESOLUTION);
        printf("hover\n");
        hover();
    }
    else if (navPoint == cur_pos->getNeighbor(RIGHT)) {
        printf("Turning right\n");
        //rotate right
        rotate(rightHeading);
        //set cur heading-ish enum
        xHeading=RIGHTH;
        //go forward
        printf("Going forward %dm\n",RESOLUTION);
        forward(RESOLUTION);
        printf("hover\n");
        hover();
        
    }
    else{
        printf("MasterI2Ccom::navToPoint: error finding direction of next point\n");
    }
    
    
}

/**
 * \brief returns the next node to go to from current discret location
 */
Node* MasterI2Ccom::getNextNavPoint( Node* finish){
    std::vector<Node*> theChosenPath;
    int pathSize;
    
    gridMap->clearParents();
    
    //TODO figure out why if I don't have this update.. a segfault is made in this funct
    cur_pos = gridMap->getNode(cur_pos->getXcoord(), cur_pos->getYcoord() );
    printf("current:");
    cur_pos->printNodeCord();
    finish = gridMap->getNode(finish->getXcoord(), finish->getYcoord() );
    printf("target:");
    finish->printNodeCord();
    ///end todo error?
    if((cur_pos!=NULL) &&(finish!=NULL)) {
        printf("neither start nor finished is NULL\n");
        theChosenPath = gridMap->findPath(cur_pos,finish); //get shortest path
        pathSize = theChosenPath.size();
        printf("got the shortest path size: %d\n", pathSize );
        if (pathSize >0) { //TODO make sure this doesn't error
            gridMap->printPath();
            printf("the chosen path is not empty, returning non-NULL\n");
            return theChosenPath.at(pathSize-1); //the path is in reverse order
        }else {
            printf("MasterI2Ccom::getNextNavPoint: shortest path is 0, no path or already there\n");
        }
    }
    return NULL;
}
//

#ifdef TEENSY_COM_TEST
int main() {
	int i;
	int testFlight;
	MasterI2Ccom com = MasterI2Ccom();	
	flight_cmd cmd = STOP;  //Just a default
	uint16_t mcmd; //
	testFlight = 1; //To keep the interface GOING!
	uint8_t dumData = 0xAA; //Dummy param's payload data to send for now

	//dum pkts
    SonarReqPkt sPkt;
	ReqPkt rPkt;
    rPkt.altitudeHold = 0; //TODO
	//open bus
	com.openi2cBus();

	printf("Lets request some Flight Commands\n");
/*
printf("ORBIT enum:%d\n",ORBIT);
printf("DOWN enum:%d\n",DOWN);
printf("UP enum:%d\n",UP);
printf("RIGHT enum:%d\n",RIGHT);
printf("LEFT enum:%d\n",LEFT);
printf("BACK enum:%d\n",BACK);
printf("FORWARD enum:%d\n",FORWARD);
printf("STOP enum:%d\n",STOP);
*/

	while(testFlight) {
        printf("NO predefined testing cmd is actually tested\n");
		printf("Options:\n[1]STOP\n[2]FORWARD\n[3]BACK\n[4]LEFT\n[5]RIGHT\n[6]UP\n[7]DOWN\n[8]ORBIT\n[9]quit\n[12]requestPkt\n");
		scanf("%d", &cmd); //might crash if non int
printf("got %d\n",cmd);
		printf("\n");
		//get some packets
		switch (cmd) {
			case 1: //supposed STOP but hold cmd
				//STOP
				/**
				rPkt.header = STOP;
				rPkt.throttle= 0x80;
				rPkt.yaw = 0x80;
				rPkt.pitch=0x80;
				rPkt.roll = 0x80;
				*/
				rPkt.header = 0x5DC;
                rPkt.throttle = 0x5DC;
                rPkt.yaw = 0x5DC;
                rPkt.pitch = 0x5DC;
                rPkt.roll = 0x5DC;
				if( com.sendPPM(&rPkt) ==1 ) { 
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
				break;
			case 2: //supposed forward cmd
				//Forward
				rPkt.header = FORWARD;
                rPkt.throttle= 0xFF;
                rPkt.yaw = 0x80;
                rPkt.pitch=0x80;
                rPkt.roll = 0x80;
				if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
				break;
			case 3: //supposed back (not really used)
				//BACK
                rPkt.header = BACK;
                rPkt.throttle= 0x00;
                rPkt.yaw = 0x80;
                rPkt.pitch=0x80;
                rPkt.roll = 0x80;
                if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
			case 4: // supposed straft left
				//Straf Left
                rPkt.header = LEFT;
                rPkt.throttle= 0x80;
                rPkt.yaw = 0x80;
                rPkt.pitch=0x80;
                rPkt.roll = 0x80;
                if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
			case 5: //supposed straf right
				//Straf Right
                rPkt.header = RIGHT;
                rPkt.throttle= 0x80;
                rPkt.yaw = 0x80;
                rPkt.pitch=0x80;
                rPkt.roll = 0x80;
                if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
			case 6: //supposed up cmd
				//UP
                rPkt.header = UP;
                rPkt.throttle= 0x80;
                rPkt.yaw = 0x80;
                rPkt.pitch=0x80;
                rPkt.roll = 0x80;

                if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
			case 7: //supposed down cmd..
				//DOWN
                rPkt.header = DOWN;
                rPkt.throttle= 0x80;
                rPkt.yaw = 0x80;
                rPkt.pitch=0x80;
                rPkt.roll = 0x80;

                if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
			case 8: //supposed orbit cmd
				//ORBIT
                rPkt.header = ORBIT;
                rPkt.throttle= 0x80;
                rPkt.yaw = 0x80;
                rPkt.pitch=0x80;
                rPkt.roll = 0x80;

                if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
			case 9: //quit program
				printf("Quitting...\n");
				testFlight =0;
				break;
			case 10:  //send set ppm pkt
				rPkt.header = 0x01;
				printf("throttle:");
				scanf("%d", &mcmd);
				rPkt.throttle = mcmd;
				printf("\nyaw:");
                scanf("%d", &mcmd);
				rPkt.yaw = mcmd;
				printf("\npitch:");
                scanf("%d", &mcmd);
                rPkt.pitch= mcmd;
				printf("\nroll:");
                scanf("%d", &mcmd);
                rPkt.roll= mcmd;
                printf("\n");
                if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
			case 11: //resend PPM pkt
				if( com.sendPPM(&rPkt) ==1 ) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("No pkt :-(\n\n");
                }
                break;
            case 12:
                if (com.requestSonar(&sPkt) ==1) {
                    printf("Got pkt \\(^_^)/\n\n");
                }else{
                    printf("no pkt :-(\n\n");
                }
                
                break;
			default:
				printf("Not supported yet\n");
				break;
		}
		while (getchar()!='\n'); //clean input buffer
	}//end flight cmd testing
        printf("Done trying for PPM packets\n\n");

#ifdef TEST_SONAR
	printf("Lets request some Sonar Readings\n");
	//get some packets
	for (i=0;i<10;i++){
		if( com.requestSonar(&sPkt) ==1 ) {
			printf("Got pkt \\(^_^)/\n\n");
		}else{
			printf("no pkt :-(\n\n");
		}
		usleep(10000);
	}
	printf("Done trying for sonar packets\n");
#endif
	//close bus
	com.closei2cBus();
	
}

#elif defined(SENSOR_TESTING)
////testing of gy-80 sensor
//int main() {
//    int dev_handle; //i2c device handle
//    int retVal;
//    int i;
//    
//    const float alpha = 0.5;
//    
//    double fXg = 0;
//    double fYg = 0;
//    double fZg = 0;
//    
//    MasterI2Ccom com = MasterI2Ccom();	//main interface
//    com.openi2cBus();                   //open i2c device
//    if ((dev_handle = com.get_dev_handle() ) >=0 ) { //if a valid handle
//        ADXL345 acc( com.get_dev_handle() ); //send the i2c device handle opened
//        
//        double pitch, roll, yaw, Xg, Yg, Zg;
//        
//        for (i=0; i<10; i++) {
//            
//            retVal = acc.read(&Xg, &Yg, &Zg);
//            
//            printf("retVal from acc.read= %d\n",retVal);
//            if (retVal >0 ) { // no error in reading device
//            
//                //Low Pass Filter
//                fXg = Xg * alpha + (fXg * (1.0 - alpha));
//                fYg = Yg * alpha + (fYg * (1.0 - alpha));
//                fZg = Zg * alpha + (fZg * (1.0 - alpha));
//            
//                //Roll & Pitch Equations
//                roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
//                pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;
//                //might be yaw
//                //yaw = (atan2()*180.0)/M_PI;
//            
//                //Serial.print(pitch);
//                printf("pitch: %f\troll:%f\n",pitch,roll);
//                //Serial.print(":");
//                //Serial.println(roll);
//            }
//            
//            
//        }//trying 10 times regardless of error
//        
//    }
//    
//    usleep(10000);
//}

//sensor testing still
int main() {
    MasterI2Ccom com = MasterI2Ccom();	//main interface
    com.openi2cBus();                   //open i2c device
    
    //TEST THE IMU
    com.startBaroThread();
    
    for (int i =0; i < 10; i++) {
        //read values 10x
        com.reqAndprintAccelerameterData();
        com.reqAndprintCompassData();
        printf("\n");
        com.reqAndprintBarometerData();
        printf("\n");
        com.reqAndprintGyrometerData();
        printf("\n\n");
        
        ::sleep(1);
    }
    //com.stopBaroThread();
    
    //TEST ROTATION
//    printf("lets test turning to 100 deg\n");
//    ::sleep (5);
//    com.rotate(100);
//    printf("lets test turning to 90 deg\n");
//    ::sleep(1);
//    com.rotate(90);
//    printf ("lets test turning to 180 deg\n");
//    ::sleep(1);
//    com.rotate(180);
//    printf ("lets test turning to 0 deg\n");
//    ::sleep(1);
//    com.rotate(0);
    
    //trying continous reading
//    com.startBaroThread();
//    ::sleep(10); //trying reading for 10 seconds
//    com.stopBaroThread();
    
    //Test launch code
     
    ::sleep (2);
    printf("Prepare to launch 1.5m w/ .1m wiggle.\n");
    com.launch(1.5, 0.1);
    //test land code
    ::sleep(2);
    printf("Prepare to land.\n");
    com.land();
    
    com.stopBaroThread();
    printf("closing\n");
    com.closei2cBus();
    
    
    return 1;
}
#elif defined(MAIN)
/**
 * \brief the actual main program for the quadcopter, skeleton
 */
int main () {
    Node* target;
    Node* next;
    int updateStatus;
    
    ///
    MasterI2Ccom nav = MasterI2Ccom();	//main interface
    nav.openi2cBus();                   //open i2c device
    //start baro continous readings
    if (nav.geti2cComStatus() & COMSTATUS_BARO){
        //ok to set baro thread for it is calibrated on online
        nav.startBaroThread();
    }
    
    printf("\n\n-----------------------------------------------\n");
    //temp cyclic executive
    for (int i= 0; i<2; i++) { //NUM_SUB_TARGETS
        //while not there...
        //get next discrete point
        printf("Getting Target %d Location for path planning\n",i+1);
        target = nav.getSubtarget (i);
        target->printNodeCord();
        //printf("getting next nav point node ptr\n");
        next = nav.getNextNavPoint(target);
        //printf("Going into while loop\n");
        while (next != NULL) {
            printf("checking if there is a map update\n");
            //update Map
            updateStatus = nav.updateMap();
            //get next point, if changed
            if (updateStatus) {
                next =nav.getNextNavPoint(target);
            }
            if (next !=NULL) {
                printf("got shortest path's next point:\t\t\t");
                
                next->printNodeCord();
                //nav to next point
                nav.navToPoint(next);
                //update cur_pos, TEMP
                nav.set_cur_pos(next);
                //update next
                next = nav.getNextNavPoint(target);
            }else{
                printf("!!!no next node to navigate to!!!\n");
            }
            
        }
        printf("---------Reached Target %d---------\n\n",i+1);
    }
    printf("Done with navigation program\n");

    nav.closei2cBus();
    return 1;
}

#elif defined (DISTANCE_LOCK)
void * demoProgramTimeOut(int runFlg) {
    
    return 0;
}

#define DISTANCE_GAP (60)
#define DISTANCE_CLOSE (30)
#define BITSHIFT (10)
int main () {
    int32_t tempT,tempP;
    //TODO use PID class to dictate controls for distance locking
    jmPID c_throttle = jmPID();
    jmPID c_pitch = jmPID();
    jmPID c_yaw = jmPID();
    jmPID c_roll = jmPID();
    
    int avoidanceDist = 1;
    int diff=0;
    
    int demo=1; // run flg
    pthread_t timingThread;
    
    //dum pkts
    SonarReqPkt sPkt;
    ReqPkt rPkt;
    
    ///
    MasterI2Ccom nav = MasterI2Ccom();	//main interface
    nav.openi2cBus();                   //open i2c device
    //start baro continous readings
    if (nav.geti2cComStatus() & COMSTATUS_BARO){
        //ok to set baro thread for it is calibrated on online
        nav.startBaroThread();
    }
    //precision
    c_throttle.bitShift(BITSHIFT);
    c_pitch.bitShift(BITSHIFT);
    c_roll.bitShift(BITSHIFT);
    c_yaw.bitShift(BITSHIFT);
    
    //The tuning stuff
    c_throttle.kPID(0.8, 0.01, 0); // Input your P, I, and D constants
    c_pitch.kPID(0.8, 0.01, 0); // Input your P, I, and D constants
   
    c_yaw.kPID(2, 0, 0); // Input your P, I, and D constants
    c_roll.kPID(2, 0, 0); // Input your P, I, and D constants
    
    //launch copter
   // nav.launch(1.5, 0.1); //lauch 1.5 meters
    
    //create timing mech
//    if (pthread_create (&timingThread,NULL,demoProgramTimeOut, demo) {
//        printf("trouble creating timing thread\nNot runing distance locking demo\n");
//        return -1;
//    }
    
    rPkt.throttle = 1600; //HOVER?
    rPkt.yaw = 1500;
    rPkt.pitch =1500;
    rPkt.roll= 1500;
    printf("MP\tSP\tThr\tPit\tRol\tYaw\n");
    while (demo) { //TODO TIMING MECH TO STOP
        // get sonar readings
        nav.requestSonar( &sPkt);
        // adjust SP from any too close sensings
        sPkt.sonar3 = 20; //TESTING DUMMY
        
        //back
        if ( (sPkt.sonar3 < DISTANCE_GAP) && (sPkt.sonar3 > 3) ) { //back sonar
            // get pid output, use constants for orignal offsets for higher convergence
            
            tempT = c_throttle.run(sPkt.sonar3, DISTANCE_GAP); // Input your P, I, and D constants
            rPkt.throttle = (uint16_t) ((int16_t) tempT + (int16_t) rPkt.throttle); //for +/- gains
            
            tempP = c_pitch.run(sPkt.sonar3,DISTANCE_GAP);
            rPkt.pitch = (uint16_t) ((int16_t) tempP + (int16_t) rPkt.pitch); //for +/- gains
            
            
            rPkt.yaw = 1500; //default pos?
            rPkt.roll = 1500; //default pos?
            // give value to actuators
            printf("%u\t%u\t%u\t%u\t%u\t%u\n",sPkt.sonar3, DISTANCE_GAP, rPkt.throttle, rPkt.pitch,rPkt.roll,rPkt.yaw);
            printf("%u\t%u\n",tempT,tempP);
            nav.sendPPM(&rPkt);
        }
//        else if((sPkt.sonar1 < 100) && (sPkt.sonar1 > 3)) { //left sonar
//            // get pid output, use constants for orignal offsets for higher convergence
//            rPkt.throttle = c_throttle.run(sPkt.sonar1, 100); // Input your P, I, and D constants
//            rPkt.pitch = 1000;//c_pitch.run(sPkt.sonar3,100);
//            rPkt.yaw = 1000; //default pos?
//            rPkt.roll = c_roll.run(sPkt.sonar1,100); //default pos?
//            // give value to actuators
//            printf("\t\t\t\tSP:%d MP:%d T:%d P:%d\n",sPkt.sonar1, 1, rPkt.throttle, rPkt.pitch);
//            nav.sendPPM(&rPkt);
//            
//        } else if((sPkt.sonar2 < 100) && (sPkt.sonar2 > 3)){ //right sonar
//            // get pid output, use constants for orignal offsets for higher convergence
//            rPkt.throttle = c_throttle.run(sPkt.sonar2, 100); // Input your P, I, and D constants
//            rPkt.pitch = 1000;//c_pitch.run(sPkt.sonar3,100);
//            rPkt.yaw = 1000; //default pos?
//            rPkt.roll = c_roll.run(sPkt.sonar2,100); //default pos?
//            // give value to actuators
//            printf("\t\t\t\tSP:%d MP:%d T:%d P:%d\n",sPkt.sonar2, 1, rPkt.throttle, rPkt.pitch);
//            nav.sendPPM(&rPkt);
//        }
        else {
            printf("Back sonar not in accountable range %dcm\n",sPkt);
        }
        ::sleep(1);
        
        //SOME STUFFS
        
        
        
    }
    
    
    
    return 1;
    
}
#elif defined (DISTANCE_LOCK_2)
#define BITSHIFT (3)
int main () {
    
    //TODO use PID class to dictate controls for distance locking
    jmPID c_throttle_r = jmPID();
    jmPID c_throttle_l = jmPID();
    jmPID c_throttle_b = jmPID();
    
    jmPID c_pitch_r = jmPID();
    jmPID c_pitch_l = jmPID();
    jmPID c_pitch_b = jmPID();
    //jmPID c_yaw = jmPID();
    jmPID c_roll_r = jmPID();
    jmPID c_roll_l = jmPID();
    jmPID c_roll_b = jmPID();
    
    
    int avoidanceDist = 1;
    int diff=0;
    
    int demo=1; // run flg
    pthread_t timingThread;
    
    //dum pkts
    SonarReqPkt sPkt;
    ReqPkt rPkt;
    
    ///
    MasterI2Ccom nav = MasterI2Ccom();	//main interface
    nav.openi2cBus();                   //open i2c device
    //start baro continous readings
    if (nav.geti2cComStatus() & COMSTATUS_BARO){
        //ok to set baro thread for it is calibrated on online
        nav.startBaroThread();
    }
    //Weird precision stuffs
    c_throttle_r.bitShift(BITSHIFT);
    c_throttle_l.bitShift(BITSHIFT);
    c_throttle_b.bitShift(BITSHIFT);
    
    c_pitch_r.bitShift(BITSHIFT);
    c_pitch_l.bitShift(BITSHIFT);
    c_pitch_b.bitShift(BITSHIFT);
    
    c_roll_r.bitShift(BITSHIFT);
    c_roll_l.bitShift(BITSHIFT);
    c_roll_b.bitShift(BITSHIFT);
    
    
    //The tuning stuff
    c_throttle_r.kPID(2, 0, 0); // Input your P, I, and D constants
    c_throttle_l.kPID(2, 0, 0); // Input your P, I, and D constants
    c_throttle_b.kPID(2, 0, 0); // Input your P, I, and D constants
    
    c_pitch_r.kPID(0.8, 0, 0); // Input your P, I, and D constants
    c_pitch_l.kPID(0.8, 0, 0); // Input your P, I, and D constants
    c_pitch_b.kPID(0.8, 0, 0); // Input your P, I, and D constants
    
    //c_yaw.kPID(6.3, 2, 0); // Input your P, I, and D constants
    c_roll_r.kPID(0.8, 0, 0); // Input your P, I, and D constants
    c_roll_l.kPID(0.8, 0, 0); // Input your P, I, and D constants
    c_roll_b.kPID(0.8, 0, 0); // Input your P, I, and D constants
    
    //launch copter
    // nav.launch(1.5, 0.1); //lauch 1.5 meters
    
    //create timing mech
    //    if (pthread_create (&timingThread,NULL,demoProgramTimeOut, demo) {
    //        printf("trouble creating timing thread\nNot runing distance locking demo\n");
    //        return -1;
    //    }
    
    while (demo) { //TODO TIMING MECH TO STOP
        // get sonar readings
        nav.requestSonar( &sPkt);
        // adjust SP from any too close sensings
        sPkt.sonar3 = 150; //TESTING DUMMY
        sPkt.sonar2 = 60;
        sPkt.sonar1 = 100;
        //back
        if ( (sPkt.sonar3 < 100) && (sPkt.sonar3 > 3) ) { //back sonar
             //not tooo close OR TOO CLOSE
        } else{
            //not adjustable
            sPkt.sonar3 = 100;
        }
        
        if((sPkt.sonar1 < 100) && (sPkt.sonar1 > 3)) { //left sonar
            //not tooo close OR TOO CLOSE
            
        } else {
            sPkt.sonar1=100;
        }
        if((sPkt.sonar2 < 100) && (sPkt.sonar2 > 3)){ //right sonar
             //not tooo close OR TOO CLOSE
        }else {
            sPkt.sonar2 =100;
        }
//        
//        else {
//            printf("Back sonar not in accountable range %dcm\n",sPkt);
//        }
        
        //SOME STUFFS
        //throttle
        //rPkt.throttle = c_throttle_r(sPkt.sonar2, c_throttle_l(sPkt.sonar1 ,c_throttle_b.run(sPkt.sonar3,100) ) );
        rPkt.throttle = c_throttle_b.run(sPkt.sonar3,100) ;
        rPkt.throttle = c_throttle_l.run(sPkt.sonar1 ,rPkt.throttle);
        rPkt.throttle =c_throttle_r.run(sPkt.sonar2, rPkt.throttle);
        
        //rPkt.pitch = c_pitch_r(sPkt.sonar2, c_pitch_l(sPkt.sonar1 ,c_pitch_b.run(sPkt.sonar3,100) ) );
        rPkt.pitch = c_pitch_b.run(sPkt.sonar3,100);
        rPkt.pitch = c_pitch_l.run(sPkt.sonar1 ,rPkt.pitch);
        rPkt.pitch = c_pitch_r.run(sPkt.sonar2,rPkt.pitch);
        
        //rPkt.roll = c_roll_r(sPkt.sonar2, c_roll_l(sPkt.sonar1 ,c_roll_b.run(sPkt.sonar3,100) ) );
        rPkt.roll = c_roll_b.run(sPkt.sonar3,100);
        rPkt.roll = c_roll_l.run(sPkt.sonar1 ,rPkt.roll);
        rPkt.roll = c_roll_r.run(sPkt.sonar2, rPkt.roll);
                            
        
        rPkt.yaw=1500;////idk
        
        printf("\t\t\t\tSB:%u SL:%u SR:%u\n",sPkt.sonar3, sPkt.sonar1, sPkt.sonar2 );
        printf("\t\t\tT:%u P:%u R:%u Y:%u\n", rPkt.throttle,rPkt.pitch,rPkt.roll,rPkt.yaw);
        ::sleep(1);
        
    }
    
    
    
    return 1;
    
}
#endif
