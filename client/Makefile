CC            =  g++
CPPFLAGS      = -Wall -g -std=c++17

# Uncomment the following line for a verbose client
#CPPFLAGS      = -Wall -g -D __UDP_CLIENT_VERBOSE__

DRIVERS = SimpleDriver.o TrackRecorderDriver.o MyDriver.o MyDriver1.o BrakeTester.o
#Put here the name of your driver class
MAIN_DRIVER = MyDriver1
#Put here the filename of your driver class header 
DRIVER_INCLUDE = '"$(MAIN_DRIVER).h"'

EXTFLAGS = -D __DRIVER_CLASS__=$(MAIN_DRIVER) -D __DRIVER_INCLUDE__=$(DRIVER_INCLUDE)

OBJECTS = WrapperBaseDriver.o SimpleParser.o CarState.o CarControl.o $(DRIVERS) 

all: $(OBJECTS) client


.SUFFIXES : .o .cpp .c

.cpp.o :
	$(CC) $(CPPFLAGS) $(EXTFLAGS) -c $<

.c.o :
	$(CC) $(CPPFLAGS) $(EXTFLAGS) -c $<


client: client.cpp $(OBJECTS)
		$(CC) $(CPPFLAGS) $(EXTFLAGS) -o client client.cpp $(OBJECTS)

clean:
	rm -f *.o client  
