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
                uint8_t header; //sensor number provided by master
                uint8_t payload; // distance recorded by requested sonar
                //uint8_t sensor2;
                //uint8_t sensor3;
                //uint8_t sensor4;
	} ReqPkt;

        /** 
         * sonar Arduino Address
         */
         int sonarArduinoAdd = 0x04;

        /** 
         * sonar Arduino Address
         */
         int ppmArduinoAdd =0x08;
#endif //SHARED_I2C_COM 
