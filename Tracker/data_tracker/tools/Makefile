CC=gcc
CXX=g++
CFLAGS = -Wall -Wextra -g -O2
CXXFLAGS = -std=c++11 -g -Wall -Wextra -O2
LIBS = -lftdi1 -linih

all: MB_daq_util process_raw.so

MB_daq_util: MB_daq_util.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

process_raw.so: process_raw.c
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $^

clean:
	rm -f *.o MB_daq_util process_raw.so
