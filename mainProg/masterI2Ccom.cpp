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
    map = new Grid (COURSE_X_DIM,COURSE_Y_DIM,RESOLUTION);
    map->initializeGrid();
    map->add_neighbors();
    subTargetList.push_back(map->getNode(T1_X,T1_Y));
    subTargetList.push_back(map->getNode(T2_X,T2_Y));
    subTargetList.push_back(map->getNode(T3_X,T3_Y));
    subTargetList.push_back(map->getNode(T4_X,T4_Y));
    
    
}

MasterI2Ccom::~MasterI2Ccom() {
	printf("~MasterI2Ccom:: Class destroyed\n");
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

	printf("requestSonar:: Received packet\n");
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
    int alpha;    //offset
    int n_1_x,n_1_y,n_2_x, n_2_y; //for mapping node coordinates
    int updateflg=0;
    
    requestSonar(&sPkt);
    
    //blklist left detection
    alpha = floor (sPkt.sonar1*100/RESOLUTION);
    n_1_x = cur_x_pos - alpha;
    n_2_x = cur_x_pos- (alpha-1);
    n_1_y = cur_y_pos;
    n_2_y = cur_y_pos;
    if ( n_1_x >= 0) { //valid nodes to remove
        n1 = map->getNode(n_1_y,n_1_y);
        n2 = map->getNode(n_2_x,n_2_y);
        if (map->removeEdge(n1,n2) ){
            printf("Removed edge from N(%d,%d) to N(%d,%d)\n",n_1_x,n_1_y,n_2_x,n_2_y);
            updateflg |=0x02;
        }
    }
    
    //blklist right detection
    alpha = floor (sPkt.sonar2*100/RESOLUTION);
    n_1_x = cur_x_pos + alpha;
    n_2_x = cur_x_pos + (alpha-1);
    n_1_y = cur_y_pos;
    n_2_y = cur_y_pos;
    if ( n_1_x <= COURSE_X_DIM) { //valid nodes to remove
        n1 = map->getNode(n_1_y,n_1_y);
        n2 = map->getNode(n_2_x,n_2_y);
        if (map->removeEdge(n1,n2) ){
            printf("Removed edge from N(%d,%d) to N(%d,%d)\n",n_1_x,n_1_y,n_2_x,n_2_y);
             updateflg |=0x04;
        }
    }
    
    //blklist back detection, not really needed?
    alpha = floor (sPkt.sonar3*100/RESOLUTION);
    n_1_x = cur_x_pos;
    n_2_x = cur_x_pos;
    n_1_y = cur_y_pos- alpha;
    n_2_y = cur_y_pos- (alpha-1);
    if ( n_1_x >= 0) { //valid nodes to remove
        n1 = map->getNode(n_1_y,n_1_y);
        n2 = map->getNode(n_2_x,n_2_y);
        if (map->removeEdge(n1,n2) ){
            printf("Removed edge from N(%d,%d) to N(%d,%d)\n",n_1_x,n_1_y,n_2_x,n_2_y);
             updateflg |=0x08;
        }
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
int forward(double meters){
    //TODO
}

/**
 * \brief keeps the craft hovering relativly in place
 */
int hover(){
    //TODO
}

//###################################END some generic flight commands


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
 
    
    return 1;
}
#endif
