#makefile for master pi to slave device communications
#comLink: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
#	g++ -o -D TEENSY_COM_TEST comLink masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

#test_sonar: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
#	g++ -D TEST_SONAR -D TEENSY_COM_TEST -o comLink masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

#test_acc: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
#	g++ -D SENSOR_TESTING -o accTest masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

#main: masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp
#	g++ -D MAIN -o lmnoCopter masterI2Ccom.cpp IMU.cpp Adafruit_LSM303.cpp Adafruit_BMP085.cpp Adafruit_L3GD20.cpp -lpthread -std=gnu++0x

#clean: 
#	rm comLink
#	rm accTest
#	rm lmnoCopter

#IDIR =../include
CC=g++
#CFLAGS=-I$(IDIR)
CFLAGS= -I -lpthread -lm -std=gnu++0x

#map files
Node.o: Node.cpp
	g++ -Wall -g -c Node.cpp -o Node.o $(CFLAGS)
Grid.o: Grid.cpp 
	g++ -c Grid.cpp -o Grid.o $(CFLAGS) 
#testing
Grid_Test.o: Grid.cpp 
	g++ -c Grid.cpp -o Grid_Test.o $(CFLAGS) -D TEST_GRID 
t_grid: Grid_Test.o Node.o
	g++ -o t_grid $(CFLAGS) Grid_Test.o Node.o 
clean_map:
	rm t_grid Node.o Grid.o

#imu files
Adafruit_BMP085.o:	Adafruit_BMP085.cpp
	 g++ -g -c Adafruit_BMP085.cpp -o Adafruit_BMP085.o $(CFLAGS) 
Adafruit_L3GD20.o:	Adafruit_L3GD20.cpp
	g++ -g -c Adafruit_L3GD20.cpp -o Adafruit_L3GD20.o $(CFLAGS)
Adafruit_LSM303.o:	Adafruit_LSM303.cpp
	g++ -g -c Adafruit_LSM303.cpp -o Adafruit_LSM303.o $(CFLAGS)
IMU.o:	IMU.cpp
	g++ -g -c IMU.cpp -o IMU.o $(CFLAGS)
	
#test
comLink: masterI2Ccom.cpp IMU.o Adafruit_LSM303.o Adafruit_BMP085.o Adafruit_L3GD20.o Grid.o Node.o
	g++ -D TEENSY_COM_TEST -g -c masterI2Ccom.cpp -o masterI2Ccom.o $(CFLAGS)
	g++ -o comLink -lpthread masterI2Ccom.o IMU.o Adafruit_LSM303.o Adafruit_BMP085.o Adafruit_L3GD20.o Node.o Grid.o $(CFLAGS)

test_sonar: masterI2Ccom.cpp IMU.o Adafruit_LSM303.o Adafruit_BMP085.o Adafruit_L3GD20.o Grid.o Node.o 
	g++ -D TEST_SONAR -D TEENSY_COM_TEST -g -c masterI2Ccom.cpp -o masterI2Ccom.o $(CFLAGS) 
	g++ -o comLink -lpthread masterI2Ccom.o IMU.o Adafruit_LSM303.o Adafruit_BMP085.o Adafruit_L3GD20.o Node.o Grid.o $(CFLAGS)

clean_comLink:
	rm masterI2Ccom.o
	rm comLink

test_acc: masterI2Ccom.cpp IMU.o Adafruit_LSM303.o Adafruit_BMP085.o Adafruit_L3GD20.o Grid.o Node.o 
	g++ -D SENSOR_TESTING -g -c masterI2Ccom.cpp -o masterI2Ccom.o $(CFLAGS) 
	g++ -o test_acc -lpthread masterI2Ccom.o IMU.o Adafruit_LSM303.o Adafruit_BMP085.o Adafruit_L3GD20.o Node.o Grid.o $(CFLAGS)
clean_test_acc:
	rm masterI2Ccom.o
	rm test_acc

#all
all: masterI2Ccom.cpp Node.o Grid.o Adafruit_BMP085.o Adafruit_L3GD20.o Adafruit_LSM303.o IMU.o
	g++ -D MAIN -g -c masterI2Ccom.cpp -o masterI2Ccom.o $(CFLAGS)
	g++ -o lmnoCopter -lpthread masterI2Ccom.o Node.o Grid.o IMU.o Adafruit_LSM303.o Adafruit_BMP085.o Adafruit_L3GD20.o $(CFLAGS)

clean:
	rm *.o
	rm lmnoCopter

#.PHONY: clean

#clean:
    #rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
