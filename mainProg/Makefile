#makefile for master pi to slave device communications
comLink: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
	g++ -o -D TEENSY_COM_TEST comLink masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

test_sonar: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
	g++ -D TEST_SONAR -D TEENSY_COM_TEST -o comLink masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

test_acc: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
	g++ -D SENSOR_TESTING -o accTest masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

main: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
	g++ -D MAIN -o lmnoCopter masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

clean: 
	rm comLink
	rm accTest
    rm lmnoCopter
