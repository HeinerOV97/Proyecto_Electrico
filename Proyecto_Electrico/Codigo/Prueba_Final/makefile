include=-I../../include
libpath=-L../../lib
libs=-lNatNet

all:build/TransmisionDatos

build/TransmisionDatos:
	g++ Transmision.cpp $(include) $(libpath) $(libs) -o build/TransmisionDatos

.PHONY: clean
clean:
	@rm -f ./build/TransmisionDatos
