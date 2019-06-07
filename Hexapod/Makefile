#############################################################################


# Makefile for Hexapod firmware


#############################################################################

prefix := /usr/local


# The recommended compiler flags for the Raspberry Pi
CXX=g++

CXXFLAGS=-I -Wall -mfpu=neon-fp-armv8 -mfloat-abi=hard -march=armv8-a -mtune=cortex-a53


# define all programs

PROGRAMS = main
DEPS = InverseKinematics
OBJ = ${PROGRAMS:=.o} ${DEPS:=.o}


all: ${PROGRAMS}
${PROGRAMS}: ${OBJ}
	${CXX} ${CXXFLAGS} -lrf24-bcm -lwiringPi $^ -o $@
	rm -rf *.o
${DEPS:=.o}: ${DEPS:=.cpp}
	${CXX} ${CXXFLAGS} $^ -c

clean:
	rm -rf *.o ${PROGRAMS}



