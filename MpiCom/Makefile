#makefile for master pi to slave device communications
comLink: masterI2Ccom.cpp
	g++ -o comLink masterI2Ccom.cpp

test_sonar: masterI2Ccom.cpp
	g++ -D TEST_SONAR -o comLink masterI2Ccom.cpp

test_acc: masterI2Ccom.cpp ADXL345.cpp
	g++ -D SENSOR_TESTING -o accTest masterI2Ccom.cpp ADXL345.cpp

clean: 
	rm comLink
	rm accTest
