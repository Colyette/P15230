#include <Wire.h>

#define SLAVE_ADDRESS 0x04
#define trigPin 13
#define echoPin 12
#define led 11
#define led2 10

int number = 0; // num pkt send back since restart...
int state = 0;
int masterCmd =0;
long distance; // distance recorded by sensor

//testing struct packet
typedef struct {
                uint8_t header;
                uint8_t sensor1;
                uint8_t sensor2;
                uint8_t sensor3;
                uint8_t sensor4;
} SonarPkt;
SonarPkt pkt;
int temp;

void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(9600);         // start serial for output
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    //Sonar pin directionality
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

//test
distance =0;

    Serial.println("Ready!");
}

void loop() {
   // delay(100);
       
     long duration;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance < 4) {  // This is where the LED On/Off happens
    digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
  digitalWrite(led2,LOW);
}
  else {
    digitalWrite(led,LOW);
    digitalWrite(led2,HIGH);
  }
  if (distance >= 200 || distance <= 0){
    Serial.println("Out of range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  delay(500);

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
         if (number = 0xAA){
           //Serial.println("Got dummy data from Pi"); 
           Serial.println(pkt.sensor1);
           Serial.println(pkt.sensor2);
           Serial.println(pkt.sensor3);
           Serial.println(pkt.sensor4);
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
  pkt.header = number++;
  pkt.sensor1 = (uint8_t) distance;
  pkt.sensor2 = (uint8_t) distance +1;
  pkt.sensor3 = (uint8_t) distance +2;
  pkt.sensor4 = (uint8_t) distance +3;
  
  Wire.write((uint8_t *)&pkt,sizeof(SonarPkt));
}

