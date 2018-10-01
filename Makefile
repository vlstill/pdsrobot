# platformio init --board uno

# Useful commands:
# `platformio run` - process/build project from the current directory
# `platformio run --target upload` or `platformio run -t upload` - upload firmware to embedded board
# `platformio run --target clean` - clean project (remove compiled files)
# `platformio run --help` - additional information

SRCS=$(wildcard src/*.cpp) $(wildcard lib/*.cpp)

all : run

build : $(SRCS)
	platformio run

run : build
	platformio run --target upload

clean :
	platformio run --target clean

monitor :
	platformio run --target monitor

.PHONY: all build run clean
