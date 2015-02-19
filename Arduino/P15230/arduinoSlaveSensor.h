/**
 * Header file for sensor slave Nano
 * 
 */

#ifndef ARDUINO_SLAVE_SENSOR
#define ARDUINO_SLAVE_SENSOR


  //########################
  //# MACROS
  //#####################

  /**
   * Sonar 1 / Left  pin config
   */
  #define TrigPin1 12
  #define EchoPin1 11

  /**
   * Sonar 2 / Right  pin config
   */
  #define TrigPin2 8
  #define EchoPin2 10

  /**
   * Sonar 3 / Rear  pin config
   */
  #define TrigPin3 7
  #define EchoPin3 9

  /**
   * Sonar 4 / Height  pin config
   */
  #define TrigPin4 13
  #define EchoPin4 6


  //########################
  //# Variables
  //#####################

  /**
   * I2C address of compass module
   */
  int compassAddress = 0x1E;

  /**
   * I2C address of barometer module
   */
   int baroAddress = 0x77;

  /**
   * Sonar 1 / Left  distance var
   */
  long distS1;

  /**
   * Sonar 2 / Right distance var
   */
  long distS2;

  /**
   * Sonar 3 / Rear 
   */
   long distS3;

  /**
   * Sonar 4 / Height 
   */
  long distS4;


#endif //Arduino_Slave_Sensor
