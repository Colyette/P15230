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
SonarPkt rPkt;
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
	::sleep(2); //wait some time
	if ((rec = ::read( dev_handle,&rPkt, 2 )) != 2  ) {
		err = errno ;
                printf("requestSonar:: Couldn't get sonar packet: errno %d\n",err);
                return -1;
	}


printf("buff: 0x%x 0x%x 0x%x 0x%x 0x%x \n", mbuff[0], mbuff[1], mbuff[2], mbuff[3],mbuff[4]);
printf("packet: 0x%d 0x%d 0x%d 0x%d 0x%d \n", rPkt.header, rPkt.sensor1, rPkt.sensor2, rPkt.sensor3,rPkt.sensor4);
	//translate to packet, mbuff
	readData = (mbuff[0] <<24) || (mbuff[1] << 16) || (mbuff[2] <<8 ) || mbuff[3];
	printf("Read 0x%x \n",readData);
	//printf("bytes: 0x%x%x%x%x \n", mbuff[0], mbuff[1], mbuff[2], mbuff[3]);

	printf("requestSonar:: Received packet\n");
	return 1;
}

int MasterI2Ccom::sendPPM(){
	//TODO send a pkt to the movement 

	printf("sendPPM:: Need to implement ppm direction framwork\n");
	return 0;
}

int main() {
	int i;
	MasterI2Ccom com = MasterI2Ccom();	

	//open bus
	com.openi2cBus();
	//get some packets
	for (i=0;i<10;i++){
		if( com.requestSonar() ==1 ) {
			printf("Got pkt \\(^_^)/\n");
		}else{
			printf("no pkt :-(\n");
		}
	}
	printf("Done trying for sonar packets\n");
	//close bus
	com.closei2cBus();
	
}
