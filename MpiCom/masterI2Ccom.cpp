/*
 *
 * @file masterI2Ccom.cpp
 * @brief  reads i2c packets from slave Arduino devices
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
#include "ADXL345.h"    //accelerameter lib




MasterI2Ccom:: MasterI2Ccom(){
	printf("MasterI2Ccom:: Class initialized \n");
	dev_handle = -1;
}

MasterI2Ccom::~MasterI2Ccom() {
	printf("~MasterI2Ccom:: Class destroyed\n");
}

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
            imu =  new IMU(dev_handle);
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
int MasterI2Ccom::requestSonar(){
	uint8_t mbuff[5]; //sizeof packet
//	ReqPkt rPkt;
	SonarReqPkt rPkt;
	int err,rec;
	uint8_t dumData = 0xAA;

    uint32_t readData;

	printf("requestSonar:: Requested packet\n");

	//point to sonar slave
	if( ioctl( dev_handle, I2C_SLAVE, sonarArduinoAdd ) < 0 ){
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
	if ((rec = ::read( dev_handle,&rPkt, sizeof(SonarReqPkt) )) != sizeof(SonarReqPkt)  ) { // sized in bytes
		err = errno ;
                printf("requestSonar:: Couldn't get sonar packet: errno %d rec: %d\n",err,rec);
                return -1;
	}


    //	printf("buff: 0x%x 0x%x 0x%x 0x%x 0x%x \n", mbuff[0], mbuff[1], mbuff[2], mbuff[3],mbuff[4]);
    //	printf("packet: 0x%d 0x%d 0x%d 0x%d 0x%d \n", rPkt.header, rPkt.sonar1, rPkt.sonar2, rPkt.sonar3,rPkt.sonar4);
    printf ("pkt #:%d, s1: %u, s2: %u, s3: %u, s4: %u\n", rPkt.header, rPkt.sonar1, rPkt.sonar2, rPkt.sonar3, rPkt.sonar4);

	//translate to packet, mbuff
	readData = (mbuff[0] <<24) || (mbuff[1] << 16) || (mbuff[2] <<8 ) || mbuff[3];
//	printf("Read 0x%x \n",readData);
	//printf("bytes: 0x%x%x%x%x \n", mbuff[0], mbuff[1], mbuff[2], mbuff[3]);

	printf("requestSonar:: Received packet\n");
	return 1;
}

//####################################
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
    imu -> getAltitude(); //TODO remove prints in getAltitude
    printf("Pressure = %d Pa\n",imu->temp );
    printf("Altitude = %f meters\n", imu->alt_m);
    printf("Pressure at sealevel (calculated) = %d Pa\n", imu->slPressure );
    printf("Real altitude = %f meters\n",  imu->r_alt_m);
}


//##########################


/**
 * \brief sends a packet to the Arduino PPM slave in order relay
 * 	a flight cmd
 * \param cmd - the command to be sent to the Arduino
 */
int MasterI2Ccom::sendPPM(ReqPkt* rPkt){
	//TODO send a pkt to the movement 
	//ReqPkt rPkt;
    int err,rec;

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

//############################################ Some generic flight commands###############################
/**
 * \brief lifts the craft from initial takeoff, could be used to get higher
 * height (use 0.1m increments)
 */
int MasterI2Ccom::launch( double height, double wiggle){
    ReqPkt pkt;
    //ensure some defaults
    pkt.header = UP;
    pkt.throttle =0;
    pkt.yaw =0;
    pkt.pitch = 0;
    pkt.roll =0;
    //iffy control
    int baro_height; //TODO sample height
    while ( baro_height != height) { //basically while forever
        if( (baro_height>=(height -wiggle)) & (baro_height<=(height+wiggle)) ){ //in an acceptable height range
            return 1; //in acceptable range
        }
        if (baro_height <= 0.010 ) { //initial lift may need to be stronger, may need sonar first
            pkt.header =UP;
            pkt.throttle= 2000; //some full throt value
            sendPPM( &pkt );
        }
        else if (baro_height > height) { //raise copter
            pkt.header = UP;
            pkt.throttle = 1500; // go up moderately
            sendPPM(&pkt);
            
        }else if (baro_height < height) { //lower copter
            pkt.header= DOWN; //over shoot some, need to release
            pkt.throttle = 1000; //
            
        }
        //TODO sample height again
    }
    
}


/**
 *  \brief lands the craft from flying position
 */
int MasterI2Ccom::land(){
    ReqPkt pkt;
    //ensure some defaults
    pkt.header = DOWN;
    pkt.throttle =0;
    pkt.yaw =0;
    pkt.pitch = 0;
    pkt.roll =0;
    //iffy control
    int baro_height; //TODO sample height
    while( baro_height >=0.1) { //while not in some safe landing distance
        //send low throttle cmds to flight controller
        pkt.header = DOWN;
        pkt.throttle =1000; //some low values
        
        //TODO sample height again
        //TODO what if something is located below (should be in object detection framework)
    }
    //TODO stop motors
    //TODO test wiggle, send ppm for att hold
    pkt.header = STOP;
    pkt.throttle = 0; //stop throttle completely
    sendPPM(&pkt);

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
    //assuming measurement already taken
    cur_pos= imu->heading;
    rChoice = cur_pos + deg;
    if (rChoice >= 360) {
        rChoice= rChoice -360; //account for range
    }
    lChoice = cur_pos - deg;
    if (lChoice < 0) {
        lChoice = lChoice+360;
    }
    //TODO take sample
    int setPos = deg; //TODO maybe plus
    wiggleL = setPos -2.5;
    if ( wiggleL <0) {
        wiggleL = wiggleL+360;
    }
    wiggleR = setPos +2.5;
    if ( wiggleR >=360){
        wiggleR = wiggleR -360;
    }
    
    while (cur_pos != setPos) { //allowing some wiggle room
        
        if ( (wiggleR >= cur_pos) && (wiggleL <= cur_pos)  ) { //good enough [+/- 2.5deg var]
            //TODO test wiggle, send ppm for att hold
            pkt.header = STOP;
            pkt.yaw = 1000; //stop rotation
            //sendPPM(&pkt);
            printf("THERE\n");
            return 1;
        } else if ( lChoice < rChoice ) { //turn left
            pkt.header = LEFT;
            pkt.yaw = 0; // going cc
            //sendPPM(&pkt);
            printf("GO LEFT\n");
            
        } else if( lChoice > rChoice) { //turn right
            pkt.header = RIGHT;
            pkt.yaw = 2000;
            //sendPPM(&pkt);
            printf("GO RIGHT\n");
        }
            
        //TODO take new sample
        imu -> getCompassValues();
        cur_pos = imu->heading;
        rChoice = cur_pos + deg;
        if (rChoice >= 360) {
            rChoice= rChoice -360; //account for range
        }
        lChoice = cur_pos - deg;
        if (lChoice < 0) {
            lChoice = lChoice+360;
        }
        ::sleep(1); // not realiable for real time.. I think (time to actualize held command)
    }
    
    //Shouldn't get here...
    return 1;
}



#ifndef SENSOR_TESTING
int main() {
	int i;
	int testFlight;
	MasterI2Ccom com = MasterI2Ccom();	
	flight_cmd cmd = STOP;  //Just a default
	uint16_t mcmd; //
	testFlight = 1; //To keep the interface GOING!
	uint8_t dumData = 0xAA; //Dummy param's payload data to send for now

	//dum pkts
	ReqPkt rPkt;

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
		printf("Options:\n[1]STOP\n[2]FORWARD\n[3]BACK\n[4]LEFT\n[5]RIGHT\n[6]UP\n[7]DOWN\n[8]ORBIT\n[9]quit\n");
		scanf("%d", &cmd); //might crash if non int
printf("got %d\n",cmd);
		printf("\n");
		//get some packets
		switch (cmd) {
			case 1: 
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
			case 2:
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
			case 3:
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
			case 4:
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
			case 5:
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
			case 6:
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
			case 7:
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
			case 8:
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
			case 9:	
				printf("Quitting...\n");
				testFlight =0;
				break;
			case 10: 
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
			case 11:
				if( com.sendPPM(&rPkt) ==1 ) {
                                        printf("Got pkt \\(^_^)/\n\n");
                                }else{
                                        printf("No pkt :-(\n\n");
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
		if( com.requestSonar() ==1 ) {
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
    
    
    for (int i =0; i < 10; i++) {
        //read values 10x
        com.reqAndprintAccelerameterData();
        com.reqAndprintCompassData();
        printf("\n");
        com.reqAndprintBarometerData();
        printf("\n\n");
        
        ::sleep(1);
    }
    
    printf("lets test turning to 100 deg\n");
    ::sleep (5);
    com.rotate(100);
    printf("lets test turning to 90 deg\n");
    ::sleep(1);
    com.rotate(90);
    printf ("lets test turning to 180 deg\n");
    ::sleep(1);
    com.rotate(180);
    printf ("lets test turning to 0 deg\n");
    ::sleep(1);
    com.rotate(0);
    
    
    printf("closing\n");
    com.closei2cBus();
    
    
    return 1;
}

#endif
