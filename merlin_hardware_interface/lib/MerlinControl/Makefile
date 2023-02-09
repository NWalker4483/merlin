CXX = g++

CPPFLAGS := -Iinclude/inc-pub -Iinclude -std=c++11 -I/home/lloyd/Desktop/AxisControl/src
CFLAGS   := -Wall# some warnings about bad code

all: 
	$(CXX) $(CPPFLAGS) -g -o a.out test.cpp -laxis_control -lsFoundation20 -lpthread
