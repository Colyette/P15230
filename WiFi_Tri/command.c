/**
 * Testing reading of terminal cmd output. Uses a subprocess to exe cmds
 * scans the files descriptor. Writes the last digit/word twice???? 
 * to compile: gcc command.c -o commandtest -lm
 * @author: Alyssa Colyette
 */
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

/**
 * @brief calculates distance given the signal level and frequency
 */
double calculateDistance(double signalLevelInDb, double freqInMHz) {
    double exp = (27.55 - (20 * log10f(freqInMHz)) + fabs(signalLevelInDb)) / 20.0;
    double result;
	printf("reading %f db, %fmhz \n",signalLevelInDb,freqInMHz);
    	result = pow(10.0,exp);
//	printf("result: %f\n",result);
    return result;
}

int main() {
 	char tempStr[25];	
	char lastStr[25];
	char substring[6];
	char * str_ptr;
	int evilNum;
	float freqNum;
	float sigNum;
	float dist;
int i;
	//for converting signal percent to dbm
	double x_RSSI;
	double x_dbm;
int MyRouter_Flag=0;

for(i=0;i<2; i++){
	//execute cmd 
	FILE *fp = popen("sudo iwlist wlan0 scan | egrep 'Cell |Frequency|Quality|ESSID'","r");
	//FILE *fp = popen("echo \"DumEFqnzy: 666\n DumEFqnzy: 555 blah\"", "r");
	//parse output
	if(fp) {
		while(!feof(fp)) { //still more data to parse
			fscanf(fp,"%s",tempStr);
			//printf("got String: %s\n", tempStr);
			if(strstr(tempStr, "Frequency") != NULL) {
			//extract frequency
				memcpy( substring, &tempStr[10], 6);
				substring[6]='\0';
				freqNum=atof(substring);
				if (freqNum !=0) {
					//if(MyRouter_Flag)
					printf("got freq:%f\n",freqNum);
				}
			}
			else if (strstr(tempStr,"level") != NULL) {
				memcpy( substring, &tempStr[6], 3);
                                substring[3]='\0';
                                sigNum=atof(substring);
                                if (sigNum !=0) {
					//if(MyRouter_Flag)
                                        printf("got Signal Level:%f of 100\n",sigNum);
					x_RSSI = sigNum/100*60;
					x_dbm = x_RSSI-95;
					dist =calculateDistance(x_dbm, freqNum*1000);
					//dist=calculateDistance(-1* (sigNum/2 +100), freqNum*1000);
					//dist =calculateDistance(-36.8, freqNum*1000);
					//if(MyRouter_Flag)
					printf("Calculated %f m\n", dist);
					MyRouter_Flag=0;
                                }
			}
			else if ( strstr(lastStr,"Address") != NULL) {
					printf("~~~~~~~~~~got Mac Address:=%s~~~~~~~~~~~~\n", tempStr);
					MyRouter_Flag=1;	//does nothing for you, me plenty
			}
			memcpy(lastStr,tempStr,25);

		}
		pclose(fp);
	}

}//end for
}
