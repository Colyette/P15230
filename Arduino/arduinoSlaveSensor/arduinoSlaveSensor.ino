#include <Wire.h>
#include <sharedi2cCom.h>

//#define SLAVE_ADDRESS 0x04
#define trigPin1 13 //trig for sensor 1
#define echoPin1 12 //echo for sensor 1
#define led 11
#define led2 10

int number = 0; // num pkt send back since restart...
int state = 0;
int masterCmd =0;
long distance1; // distance recorded by sensor1

/*
//testing struct packet
typedef struct {
                uint8_t header;
                uint8_t sensor1;
                uint8_t sensor2;
                uint8_t sensor3;
                uint8_t sensor4;
} SonarPkt;
*/
ReqPkt pkt;
int temp;

void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(9600);         // start serial for output
    // initialize i2c as slave
    Wire.begin(sonarArduinoAdd);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    //Sonar pin directionality
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);

//test
distance1 =0;

    Serial.println("Ready!");
}

void loop() {
   // delay(100);
       
     long duration;
  digitalWrite(trigPin1, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin1, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin1, LOW);
  duration = pulseIn(echoPin1, HIGH);
  distance1 = (duration/2) / 29.1;
  if (distance1 < 4) {  // This is where the LED On/Off happens
    digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
  digitalWrite(led2,LOW);
}
  else {
    digitalWrite(led,LOW);
    digitalWrite(led2,HIGH);
  }
  if (distance1 >= 200 || distance1 <= 0){
    Serial.println("Out of range");
  }
  else {
    Serial.print(distance1);
    Serial.println(" cm");
  }
  delay(20); /// sensor reads 3.06m in 18ms

}

// callback for received data
void receiveData(int byteCount){

    while(Wire.available()) {
        //number = Wire.read();
        masterCmd = Wire.read();
        Serial.print("data received: ");
        Serial.println(number);

        if (number == 1){

            if (state == 0){
                digitalWrite(13, HIGH); // set the LED on
                state = 1;
            }
            else{
                digitalWrite(13, LOW); // set the LED off
                state = 0;
            }
         }
         
         /// test for dummydata
         if (masterCmd = 0xAA){
           //Serial.println("Got dummy data from Pi"); 
           Serial.println(pkt.payload);
           //Serial.println(pkt.sensor2);
           //Serial.println(pkt.sensor3);
           //Serial.println(pkt.sensor4);
         }
     }
}

// callback for sending data
void sendData(){
    //Wire.write(number);
    //Wire.write(distance); // want to write out last read distance
    
    //Wire.beginTransmission(SLAVE_ADDRESS);
    //Wire.write(number);
    //Wire.write(number+1);
    //Wire.write(number+2);
    //Wire.write(number+3);
    //Wire.endTransmission();
  /*  
  pkt.header = distance++ ;
  pkt.sensor1 = (uint8_t) number;
  pkt.sensor2 = (uint8_t)number +1;
  pkt.sensor3 = (uint8_t)number +2;
  pkt.sensor4 = (uint8_t)number +3;
  */
  
  //send sor
  pkt.header = number++;
  pkt.payload = (uint8_t) distance1;
  //pkt.sensor2 = (uint8_t) distance +1;
  //pkt.sensor3 = (uint8_t) distance +2;
  //pkt.sensor4 = (uint8_t) distance +3;
  
  Wire.write((uint8_t *)&pkt,sizeof(ReqPkt));
}

