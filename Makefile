CXX=g++
CXXFLAGS=-W -Wall -g -fmessage-length=0 -std=gnu++11
OPTIMIZE=-O2 -fomit-frame-pointer
DEBUG=-O0 -lmcheck

all: marlinfeed mocklin

test: unit-tests
	unit-tests

%: src/%.cpp src/marlinbuf.h src/gcode.h src/file.h src/fifo.h src/dirscanner.h
	$(CXX) $(CXXFLAGS) $(OPTIMIZE) -o $@ $<

unit-tests: src/unit-tests.cpp src/marlinbuf.h src/gcode.h src/file.h src/fifo.h src/dirscanner.h
	$(CXX) $(CXXFLAGS) $(DEBUG) -o $@ $<

mocklin: src/mocklin.cpp src/marlinbuf.h src/gcode.h src/file.h
	$(CXX) $(CXXFLAGS) $(DEBUG) -o $@ $<

clean:
	rm -f marlinfeed unit-tests mocklin
	rm -f *~
