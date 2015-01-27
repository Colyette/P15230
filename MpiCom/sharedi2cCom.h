/**
 * @brief header file with slave addresses and packet structures 
 * for RPi and Arduino communications
 * @author Alyssa Colyette
 */
#ifndef SHARED_I2C_COM
#define SHARED_I2C_COM

	/**
         * sonar pkt struct
         */
	typedef struct {
                uint8_t header;
                uint8_t sensor1;
                uint8_t sensor2;
                uint8_t sensor3;
                uint8_t sensor4;
	} SonarPkt;

        /** 
         * sonar Arduino Address
         */
         int sonarArduinoAdd = 0x04;

        /** 
         * sonar Arduino Address
         */
         int ppmArduinoAdd;
#endif //SHARED_I2C_COM 
