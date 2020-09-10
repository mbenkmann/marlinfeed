CXX=g++
CXXFLAGS=-W -Wall -g -fmessage-length=0 -std=gnu++11
OPTIMIZE=-O2 -fomit-frame-pointer
DEBUG=-O0 -lmcheck

all: marlinfeed marlinfeed.1

test: unit-tests
	./unit-tests

%: src/%.cpp src/marlinbuf.h src/gcode.h src/file.h src/fifo.h src/dirscanner.h
	$(CXX) $(CXXFLAGS) $(OPTIMIZE) -o $@ $<

unit-tests: src/unit-tests.cpp src/marlinbuf.h src/gcode.h src/file.h src/fifo.h src/dirscanner.h
	$(CXX) $(CXXFLAGS) $(DEBUG) -o $@ $<

mocklin: src/mocklin.cpp src/marlinbuf.h src/gcode.h src/file.h
	$(CXX) $(CXXFLAGS) $(DEBUG) -o $@ $<

marlinfeed.1: README.md
	go-md2man -in=$< -out=$@

.PHONY: debian
debian:
	dpkg-buildpackage -rfakeroot -sa -uc -us

clean:
	rm -f marlinfeed unit-tests mocklin marlinfeed.1
	rm -f *~
