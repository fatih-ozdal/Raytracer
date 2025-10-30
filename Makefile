CXX = g++

CXXFLAGS = -std=c++11 -Iinclude -O3 -fopenmp
SRC = $(wildcard *.cpp)
TARGET = raytracer

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

debug: $(SRC)
	$(CXX) $(CXXFLAGS) -g  $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
