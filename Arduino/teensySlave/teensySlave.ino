/**
 * P15230_Teensy_Rev1.ino
 *
 * 
 *   
 * \author Alyssa Colyette
 * \author Philip Tifone
 * \author James Mussi
 */

// Include sections
#include <Wire.h>
#include <sharedi2cCom.h>
#include <PulsePosition.h> 
#include <IntervalTimer.h>


#define TRIG 2
//#define trig2 12
#define ECHO_L 3
#define ECHO_R 4
#define ECHO_Bk 5
#define ECHO_Lo 6
#define MUX_SEL 7
#define MUX_ENA 8
#define RX_IN 10
#define B_LED 13
#define VOLT_IN A1
#define CURR_IN A0
#define CHANNEL_NUM 6


volatile uint32_t swBegin = 0 ;
volatile uint32_t swLength = 1500 ;
volatile uint32_t echoBegin_L = 0 ;
volatile uint32_t echoLength_L = 0 ;
volatile uint32_t echoBegin_R = 0 ;
volatile uint32_t echoLength_R = 0 ;
volatile uint32_t echoBegin_Bk = 0 ;
volatile uint32_t echoLength_Bk = 0 ;
volatile uint32_t echoBegin_Lo = 0 ;
volatile uint32_t echoLength_Lo = 0 ;
uint16_t voltSample = 0;                                      // value of voltage sample
uint16_t currSample = 0;                                      // value of current sample
uint32_t time2wait=0;                                           // ping sensor delay time
uint16_t distance_L = 0;                      // distance of object to left of craft (cm)
uint16_t distance_R = 0;                      // distance of object to right of craft (cm)
uint16_t distance_Bk = 0;                     // distance of object to back of craft (cm)
uint16_t distance_Lo = 0;                     // distance of floor to bottom of craft (cm)



uint16_t flg ;                                                    // flag for printing, only for testing
uint16_t sFlg;                                                      // set for a sonar request
uint16_t rFlg;                                                          // sonar request pkt is ready to send at read
volatile uint16_t pFlg=0;                                                   // flag for new ping data available
uint16_t aFlg;                                                  // flag for new adc data available                          
uint16_t swFlg;                              // flag for new switch timing

ReqPkt pkt;                                                         // packet to be received and passed via I2C 
SonarReqPkt spkt;                                               // sonar request packet

PulsePositionOutput myOutput(FALLING);              // declare PPM object
IntervalTimer pingTimer;                        // declare timing object 
IntervalTimer adcTimer; 
IntervalTimer ppmTimer; 

void setup() {
  Serial.begin(9600);                                        // DEBUG: start serial for output 
  Wire.begin(ppmArduinoAdd);                                 // initialize i2c as slave
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  // begin the PPM Stream to the flight controller
  myOutput.begin(CHANNEL_NUM);
  // begin the interval timer
  pingTimer.begin(PingSensor, 10000);                     // Call the PingSensor function 10 milli
  adcTimer.begin(ADC, 1000000);                            // Sample voltage & Current  1000 milli
  ppmTimer.begin(updatePPM, 20000);                      // update the ppm values every 20 milli seconds
  pingTimer.priority(120);
  adcTimer.priority(200);
  ppmTimer.priority(100);                                      // Set slightly higher than avg priority for my tasks    
  // set i/o for hardware
  pinMode(TRIG, OUTPUT);                       // Trigger pin set to output
  //pinMode(trig2, OUTPUT);                       // Trigger pin2 set to output
  pinMode(ECHO_L, INPUT);                      // Left ping sensor echo pin set to input
  pinMode(ECHO_R, INPUT);                      // Right ping sensor echo pin set to input
  pinMode(ECHO_Bk, INPUT);                     // Rear ping sensor echo pin set to input
  pinMode(ECHO_Lo, INPUT);                     // Bottom ping sensor echo pin set to input
  pinMode(B_LED, OUTPUT);                      // Board LED set to output
  pinMode(RX_IN, INPUT);                       // Reciever input (Overide Switch) 
  pinMode(MUX_SEL, OUTPUT);
  pinMode(MUX_ENA, OUTPUT);
  // declare interrupt service routines
  attachInterrupt(RX_IN, swService, CHANGE);          // Switch interupt routeen 
  attachInterrupt(ECHO_L, echoService_L, CHANGE);     // Left ping sensor interupt routeen
  attachInterrupt(ECHO_R, echoService_R, CHANGE);     // Right ping sensor interupt routeen
  attachInterrupt(ECHO_Bk, echoService_Bk, CHANGE);   // Back ping sensor interupt routeen
  attachInterrupt(ECHO_Lo, echoService_Lo, CHANGE);   // Rear ping sensor interupt routeen
  //defaults  
  Serial.println(" Setup Routeen Finished");                    // Teensy debugging (Remove in final code)
  Serial.println("...Ready!...");                      // Debug
  }

  /**
   * \brief basically a cyclic executive to run all streams?
   */
  void loop() {
    if (swFlg>= 1){
      channelSet();
    }
    if (pFlg >= 15){
      pingAvg();
      //Serial.println("wtf"); // 4debug flag
      pFlg=0;
    }
    printer();     // printing function for debugging    
  }

/*
 * Subrouteens Live Here :-)
 */
void updatePPM(){

  myOutput.write(1, pkt.throttle); 
  myOutput.write(2, pkt.yaw); 
  myOutput.write(3, pkt.pitch); 
  myOutput.write(4, pkt.roll); 
  myOutput.write(5, echoLength_L);
  myOutput.write(6, swLength);                // Updates PPM Function
}       

void PingSensor(){
  // To Trigger all the ping sensors!!!!! THIS IS DONE TO TRIGGER A BATCH READING, IF ERRORS DEVELOPE: CHANGE TO SEQUENTIAL TRIGGERING
  digitalWrite(TRIG, HIGH);                      // start trigger pulse Left
  //digitalWrite(trig2, HIGH);
  time2wait = (micros()+10);                     // time in future to end trigger pulse
  while(micros() < time2wait) {                // waiting!!!
    //hello world
  }
  digitalWrite(TRIG, LOW);                     // end the trigger pulse Left                         // Triggers Ping Sensors
  //digitalWrite(trig2, LOW);
}


void ADC(){
  voltSample = analogRead(VOLT_IN);           // Voltage ADC
  currSample = analogRead(CURR_IN);           // Current ADC
  aFlg = 1 ;                                          // set new ADC available                                 // Voltage & Current Sense                                 // Triggers ADC
} 

void channelSet(){
  if(swLength >= 1500){
    digitalWrite(MUX_SEL, LOW);
  }
  else{
    digitalWrite(MUX_SEL, HIGH);
  }
  swFlg = 0;                        // Sets the Mux Sel pin (Human or CPU Control)
}

void pingAvg(){                              // Converts & Filters Ping data
  distance_L = (echoLength_L / 58);          // Computes distance from Left ping sensor in CM
  distance_R = (echoLength_R / 58);          // Computes distance from Right ping sensor in CM
  distance_Bk = (echoLength_Bk / 58);        // Computes distance from Back ping sensor in CM
  distance_Lo = (echoLength_Lo / 58);        // Computes distance from Bottom ping sensor in CM
  //pFlg=0;
  spkt.sonar1 = distance_L;
  spkt.sonar2 =distance_R;
  spkt.sonar3 = distance_Bk;
  spkt.sonar4 =distance_Lo;
  
  // Hello World
}

void printer(){                               // Sends serial data for Debugging
  uint32_t peetime2wait=0;
  peetime2wait = (micros()+500000);           // time in future to end trigger pulse
  while(micros() < peetime2wait) {            // waiting!!!
  //hello world
  }
  Serial.print("PLength Right: ");
  Serial.println(echoLength_R);
  Serial.print("Distance Right(cm): ");
  Serial.println(distance_R);
  Serial.print("PLength Left: ");
  Serial.println(echoLength_L);
  Serial.print("Distance Left(cm): ");
  Serial.println(distance_L);
  Serial.print("PLength Back: ");
  Serial.println(echoLength_Bk);
  Serial.print("Distance Back(cm): ");
  Serial.println(distance_Bk);
  Serial.print("PLength Bottom: ");
  Serial.println(echoLength_Lo);
  Serial.print("Height(cm): ");
  Serial.println(distance_Lo);
  Serial.print("Switch Length: ");
  Serial.println(swLength);
  Serial.println("...END...");
}


/*
 * Interupts Live Here
 */
void swService(){                             // switch interupt 
  cli();                                        // disable interupts
  if (digitalReadFast(RX_IN) == HIGH){          // Rising edge? Yes
    swBegin = micros();                         // Capturing rising edge
  }
  else {                                        // No
    swLength = (micros() - swBegin);            // compute length of pulse
  }
  swFlg = 1;                                    // new switch timing available
  sei();                                        // enable interupts
}

void echoService_L(){                         // Left ping sensor interput
  cli();                                       // disable interupts
  if (digitalReadFast(ECHO_L)==HIGH){          // Rising edge? Yes
    echoBegin_L = micros();                    // Capture rising edge
 //   digitalWrite(B_LED, HIGH);                               // Flashes B_LED for debugging               
  }
  else{                                        // No
    echoLength_L = (micros() - echoBegin_L);   // compute length of pulse
 //   digitalWrite(B_LED, LOW);                              // Flashes B_LED for debugging 
  }
  pFlg |= (0x0001);
  //pFlg=1;                                                // set bit 0 of pFlg to 1
  sei();                                       // enable interupts
}
void echoService_R(){                         // Right ping sensor interupt
  cli();                                       // dissable interupts
  if (digitalReadFast(ECHO_R)==HIGH){          // Rising edge? yes
    echoBegin_R = micros();                    // Capture rising edge
    digitalWrite(B_LED, HIGH);                             // Flashes B_LED for debugging               
  }
  else{                                        // No
    echoLength_R = (micros() - echoBegin_R);   // Compute lenght of pulse
    digitalWrite(B_LED, LOW);                                // Flashes B_LED for debugging 
  }
  pFlg |= (0x0002);                                                // set bit 1 of pFlg to 1 
  sei();                                       // enable interupts
}
void echoService_Bk(){                        // Back ping sensor interupt
  cli();                                       // disable interupts
  if (digitalReadFast(ECHO_Bk)==HIGH){         // Rising edge? yes
    echoBegin_Bk = micros();                   // Caputre rising edge
    // digitalWrite(B_LED, HIGH);                            // Flashes B_LED for debugging             
  }
  else{                                        // No
    echoLength_Bk = (micros() - echoBegin_Bk); // compute length of pulse
    // digitalWrite(B_LED, LOW);                             // Flashes B_LED for debugging 
  }
  pFlg |= (0x0004);                                                // set bit 2 of pFlg to 1
  sei();                                       // enable interupts
}
void echoService_Lo(){                        // Bottom ping sensor interupt
  cli();                                       // disable interupts
  if (digitalReadFast(ECHO_Lo)==HIGH){         // Rising edge? Yes
    echoBegin_Lo = micros();                   // Caputre rising edge
    digitalWrite(B_LED, HIGH);                             // Flashes B_LED for debugging               
  }
  else{                                        // No
    echoLength_Lo = (micros() - echoBegin_Lo); // compute length of pulse 
    digitalWrite(B_LED, LOW);                                // Flashes B_LED for debugging 
  }
  sei();
  pFlg |= (0x0008);                                                // set bit 3 of pFlg to 1
}



/*
 * \brief callback for re11ceived data
 */
void receiveData(int byteCount){
  //Serial.println(byteCount);
  while(Wire.available()) {
    //number = Wire.read();
    digitalWrite(13, HIGH);
    if(byteCount == sizeof(ReqPkt) ) { //only reads a byte at a time
      pkt.header = Wire.read(); //LSB read first
      pkt.header |= (Wire.read() <<0x8);


      pkt.throttle = Wire.read();
      pkt.throttle |= (Wire.read() << 8);
    
      pkt.yaw = Wire.read();
      pkt.yaw |= (Wire.read() <<0x8);
    
      pkt.pitch = Wire.read();
      pkt.pitch |= (Wire.read() << 0x8);
    
      pkt.roll = Wire.read();
      pkt.roll |= (Wire.read() << 0x8);
      flg =1;
    }
   } //expectd pkt size

}


/*
 * \brief callback for sending data
 * not sure what reply should be sent back as an ack
 */
void sendData(){

  //edit reply pkt
  //pkt.header = 
  //pkt.payload = (uint8_t) distance1;

  //Wire.beginTransmission(sonarArduinoAdd);
  //if (rFlg) {
  Wire.write((uint8_t *)&spkt,sizeof(SonarReqPkt));
  //rFlg=0;
  //}
  //else {
  //  Wire.write((uint8_t *)&pkt,sizeof(ReqPkt));
  //}
}




