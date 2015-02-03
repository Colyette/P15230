/**
 * \brief header file with slave addresses and packet structures 
 * for RPi and Arduino communications
 * \author Alyssa Colyette
 * Contact: axc4954@rit.edu
 */
#ifndef SHARED_I2C_COM
#define SHARED_I2C_COM

	/**
	 * ReqPkt
         * \brief Request packet struct, header is the cmd from the Pi
 	 * and the request is interpreted by the slave Arduino. The payload
	 * serves as the reply space for the Arduino but can also function 
	 * as an extra parameter for the Pi to the Arduino
         */
	typedef struct {
                uint8_t header; //Command to the microcontroller
                uint8_t payload; //For extra params, or requested data
	} ReqPkt;

        /** 
         * \brief sonar Arduino Address
         */
         int sonarArduinoAdd = 0x04;

        /** 
         * \brief sonar Arduino Address
         */
         int ppmArduinoAdd =0x08;

	/**
	 * Flight commands
	 * \brief These are the header commands recognizable by the Flight 
	 * Arduino 
	 */
	typedef enum {STOP, FORWARD, BACK, LEFT, RIGHT, UP, DOWN, ORBIT} flight_cmd; 

	/**
	 * Sensor Commands
	 * \brief These are the header commands recognizable by the Sensor
	 * Arduino. Each command is a sensor request
	 */
	typedef enum {SONAR1, SONAR2, SONAR3, SONAR4, COMPASS, BAROMETER} sensor_cmd;

#endif //SHARED_I2C_COM 
