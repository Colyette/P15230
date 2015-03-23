#makefile for master pi to slave device communications
comLink: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp
	g++ -o comLink masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp

test_sonar: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp
	g++ -D TEST_SONAR -o comLink masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp

test_acc: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp
	g++ -D SENSOR_TESTING -o accTest masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp -lpthread 

clean: 
	rm comLink
	rm accTest
