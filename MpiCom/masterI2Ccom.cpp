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
#include "masterI2Ccom.h" 
#include "sharedi2cCom.h" 

MasterI2Ccom:: MasterI2Ccom(){
	printf("MasterI2Ccom:: Class initialized \n");
	dev_handle = -1;
}

MasterI2Ccom::~MasterI2Ccom() {
	printf("~MasterI2Ccom:: Class destroyed\n");
}

int MasterI2Ccom::openi2cBus() {
	if(dev_handle > 0) {
		printf("openi2cBus:: I2C bus already opened\n");
		return 0;
	}else{
		if( ( dev_handle = ::open("/dev/i2c-1", O_RDWR) ) < 0 ){
			printf("openi2cBus:: Couldn't open I2C bus\n") ;
			
			// initial open failed, don't bump the ref count
			return -1 ;
		}
	}
	printf("openi2cBus:: Bus opened desc=%d\n",dev_handle);
 	return 1;
}

int MasterI2Ccom::closei2cBus() {
	//TODO close linux driver
	int ret;
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
printf("other heading:%u, altitude:%u \n", rPkt.heading, rPkt.altitude);
	//translate to packet, mbuff
	readData = (mbuff[0] <<24) || (mbuff[1] << 16) || (mbuff[2] <<8 ) || mbuff[3];
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

	//uint32_t readData;

        //printf("sendPPM:: Requested packet %d\n", cmd);
//	rPkt.header = cmd;
//	rPkt.payload = param;

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

        printf("requestSonar:: Received packet\n");
	return 1;
}

int main() {
	int i;
	int testFlight;
	MasterI2Ccom com = MasterI2Ccom();	
	flight_cmd cmd = STOP;  //Just a default
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
