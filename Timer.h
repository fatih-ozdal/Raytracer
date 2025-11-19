#include <chrono>
#include <iostream>

#include <string>

class Timer {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    
public:
    Timer() : start_time(std::chrono::high_resolution_clock::now()) {}
    
    void reset() {
        start_time = std::chrono::high_resolution_clock::now();
    }
    
    double elapsed() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(end_time - start_time).count();
    }

    void printElapsed(string message) const {
        std::cout << message << " Elapsed: " << elapsed() << " ms" << std::endl;
    }
};