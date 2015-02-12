#include <Wire.h>
#include <sharedi2cCom.h>
#include <arduinoSlaveSensor.h>

//#define SLAVE_ADDRESS 0x04
//#define TrigPin1 12 //trig for sensor 1
//#define EchoPin1 11 //echo for sensor 1
//#define TrigPin2 8
//#define EchoPin2 10
//#define led 13
//#define led2 9



int number = 0; // num pkt send back since restart...
int state = 0;
int masterCmd =0;
long distance1; // distance recorded by sensor1
long distance2,distance3,distance4;
uint16_t buffer[7];


SonarReqPkt pkt;
int temp;

void setup() {
    Serial.begin(9600);         // start serial for output
    // initialize i2c as slave
    Wire.begin(sonarArduinoAdd);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    //Sonar pin directionality
    pinMode(TrigPin1, OUTPUT);
    pinMode(EchoPin1, INPUT);
    pinMode(TrigPin2,OUTPUT);
    pinMode(EchoPin2,INPUT);
    pinMode(TrigPin3,OUTPUT);
    pinMode(EchoPin3,INPUT);
    pinMode(TrigPin4,OUTPUT);
    pinMode(EchoPin4,INPUT);

  //default sonar distances calculated test  
  distance1 =0;
  distance2=0;
  distance3=0;
  distance4=0;

    Serial.println("Ready!");
}


void pingSonar1() {
   long duration;
   
  //sensor 1 Left
  digitalWrite(TrigPin1, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(TrigPin1, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(TrigPin1, LOW);
  duration = pulseIn(EchoPin1, HIGH,1900); // 3rd param, time us
  distance1 = (duration/2) / 29.1;
/* //LED STATUS
  if (distance1 < 4) {  // This is where the LED On/Off happens
    //digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
    //digitalWrite(led2,LOW);
  }
  else {
    digitalWrite(led,LOW);
    digitalWrite(led2,HIGH);
  }
*/
  if (distance1 >= 200 || distance1 <= 0){
  //  Serial.println("Sensor 1 Left: Out of range"); //recycles last dist cal?
  }
  else {
 /*  Serial.print("Sensor 1 Left: ");
    Serial.print(distance1);
    Serial.println(" cm");
*/ 
  }
}

void pingSonar2 () {
  long duration;
  
  //Sensor 2 Right
  digitalWrite(TrigPin2, LOW);  // make low before pulse
  delayMicroseconds(2); // recognize it low
  digitalWrite(TrigPin2, HIGH); // Trigger 10us pulse
  delayMicroseconds(10); 
  digitalWrite(TrigPin2, LOW);
  duration = pulseIn(EchoPin2, HIGH,19000); //Check for echo pulse 3rd param wait time
  distance2 = (duration/2) / 29.1; //calc distance

  if (distance2 >= 200 || distance2 <= 0){
////    Serial.println("Sensor 2 Right: Out of range");
  }
  else {
/*   Serial.print("Sensor 2 Right: ");
    Serial.print(distance2);
    Serial.println(" cm");
 */
  }
     
}


void pingSonar3 () {
  long duration;
  
  //Sensor 2 Right
  digitalWrite(TrigPin3, LOW);  // make low before pulse
  delayMicroseconds(2); // recognize it low
  digitalWrite(TrigPin3, HIGH); // Trigger 10us pulse
  delayMicroseconds(10); 
  digitalWrite(TrigPin3, LOW);
  duration = pulseIn(EchoPin3, HIGH,19000); //Check for echo pulse 3rd param wait time
  distance3 = (duration/2) / 29.1; //calc distance

  if (distance3 >= 200 || distance3 <= 0){
////    Serial.println("Sensor 3 Right: Out of range");
  }
  else {
/*   Serial.print("Sensor 3 Right: ");
    Serial.print(distance3);
    Serial.println(" cm");
 */
  }
     
}

void pingSonar4 () {
  long duration;
  
  //Sensor 2 Right
  digitalWrite(TrigPin4, LOW);  // make low before pulse
  delayMicroseconds(2); // recognize it low
  digitalWrite(TrigPin4, HIGH); // Trigger 10us pulse
  delayMicroseconds(10); 
  digitalWrite(TrigPin4, LOW);
  duration = pulseIn(EchoPin4, HIGH,19000); //Check for echo pulse 3rd param wait time
  distance4 = (duration/2) / 29.1; //calc distance

  if (distance4 >= 200 || distance4 <= 0){
////    Serial.println("Sensor 4 Right: Out of range");
  }
  else {
/*   Serial.print("Sensor 4 Right: ");
    Serial.print(distance4);
    Serial.println(" cm");
 */
  }
     
}

void pingAllSensors () {
  pingSonar1();
  delay(10); /// sensor reads 3.06m in 18ms
  pingSonar2();
  delay(10); /// sensor reads 3.06m in 18ms
  pingSonar3();
  delay(10); /// sensor reads 3.06m in 18ms
  pingSonar4();
  delay(10); /// sensor reads 3.06m in 18ms
}

void loop() {
  pingAllSensors();
}

// callback for received data
void receiveData(int byteCount){

    while(Wire.available()) {
        //number = Wire.read();
        masterCmd = Wire.read();
        Serial.print("data received: ");
        Serial.println(masterCmd);
Serial.println(sizeof(SonarReqPkt));
         
         /// test for dummydata
         if (masterCmd = 0xAA){
           Serial.println("Got dummy data from Pi"); 
           Serial.println(pkt.sonar1);
           Serial.println(pkt.sonar2);
           Serial.println(pkt.sonar3);
           Serial.println(pkt.sonar4);
         }
     }
}

// callback for sending data
void sendData(){
  
  //send sor
  
  pkt.header = (uint16_t) number++;
  //pkt.payload = (uint8_t) distance1;
  pkt.sonar1 =  (uint16_t) distance1;
  pkt.sonar2 = (uint16_t) distance2;
  pkt.sonar3 = (uint16_t) distance3;
  pkt.sonar4 = (uint16_t) distance4;
  pkt.heading = (uint16_t) 0xAE;
  pkt.altitude = (uint16_t) 0xAF;
  
  //Wire.beginTransmission(sonarArduinoAdd);
  Wire.write((uint8_t *)&pkt,sizeof(SonarReqPkt));
 
}

