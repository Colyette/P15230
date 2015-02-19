/**
 * arduinoSlavePPM.ino
 *
 * \brief handles requests for flight command updates from the Pi 
 *   
 * \author Alyssa Colyette
 * \author Philip Tifone
 */
 
#include <Wire.h>
#include <sharedi2cCom.h>
//#include <arduinoSlaveSensor.h>

int number = 0; // num pkt send back since restart...
int state = 0;
int masterCmd =0; //the header read from the master device
uint16_t buffer[7];

int flg; 

ReqPkt pkt; //packet to be received and passed via I2C 
int temp;

void setup() {
    Serial.begin(9600);         // start serial for output
    // initialize i2c as slave
    Wire.begin(ppmArduinoAdd);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    //pin directionality
  
    //defaults  
  
    Serial.println("Ready!");
}




/**
 * \brief basically a cyclic executive to run all streams?
 */
void loop() {
  //int flg;
  if(flg) {
    //Serial.println(sizeof(ReqPkt));
     Serial.println(pkt.header);
       // Serial.print("\nthrottle: ");
      Serial.println(pkt.throttle);
    //    Serial.print("\nyaw: ");
      Serial.println(pkt.yaw);
   //     Serial.print("\npitch: ");
      Serial.println(pkt.pitch);
    //    Serial.print("\nroll: ");
      Serial.println(pkt.roll);
      flg = 0; 
  }
  
}

/*
 * \brief callback for re11ceived data
 */
void receiveData(int byteCount){
  //Serial.println(byteCount);
    while(Wire.available()) {
        //number = Wire.read();
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
        } //expectd pkt size
       
        //testing prints, DO NOT keep in actual implementation
  
  //      Serial.print("data received::\nCmd: ");
     //   Serial.println(pkt.header);
       // Serial.print("\nthrottle: ");
       // Serial.println(pkt.throttle);
    //    Serial.print("\nyaw: ");
        //Serial.println(pkt.yaw);
   //     Serial.print("\npitch: ");
        //Serial.println(pkt.pitch);
    //    Serial.print("\nroll: ");
        //Serial.println(pkt.roll);
        
        
//Serial.println(sizeof(ReqPkt));
               
     }
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
  Wire.write((uint8_t *)&pkt,sizeof(ReqPkt));
 
}

