SRCS=$(wildcard src/*.cpp) $(wildcard lib/*.cpp)

all : run

build : $(SRCS)
	idf build

run : build
	idf flash

monitor :
	idf monitor

.PHONY: all build run clean
