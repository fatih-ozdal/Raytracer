CXX = g++

CXXFLAGS = -std=c++11 -Iinclude
SRC = $(wildcard *.cpp)
TARGET = raytracer

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -fopenmp -O3 $(SRC) -o $(TARGET)

debug: $(SRC)
	$(CXX) $(CXXFLAGS) -g -O1 $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
