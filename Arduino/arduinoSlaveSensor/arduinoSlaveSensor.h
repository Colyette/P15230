/**
 * Header file for sensor slave Nano
 * 
 */

#ifndef ARDUINO_SLAVE_SENSOR
#define ARDUINO_SLAVE_SENSOR

/**
 * I2C address of compass module
 */
 compassAddress= 0x1E;

/**
 * I2C address of barometer module
 */
 baroAddress= 0x77;

/**
 * Sonar 1 / Left 
 */
long distS1;
#define TrigPin1 = 12;
#define EchoPin1 = 11;


/**
 * Sonar 2 / Right
 */
long distS2;
#define TrigPin2 = 8;
#define EchoPin2 = 10;

/**
 * Sonar 3 / Rear 
 */
 long distS3;
#define TrigPin3 = 7;
#define EchoPin3 = 9;

/**
 * Sonar 4 / Height 
 */
long distS4;
#define TrigPin1 = 13;
#define EchoPin1 = 6;


#endif //Arduino_Slave_Sensor
