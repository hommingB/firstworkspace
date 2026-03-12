#pragma once

#include <string>
#include <cmath>

class Wheel
{
public:
    std::string name = "";
    int enc = 0;
    int enc_prev = 0;
    double pos = 0.0;
    double vel = 0.0;

    double cmd = 0.0;
    double rads_per_count = 0.0;
    Wheel() = default;
    Wheel(const std::string& wheel_name, int resolution) {
        setup(wheel_name, resolution);
    }
    ~Wheel() = default;

    void setup(const std::string& wheel_name, int resolution){
        name = wheel_name;
        rads_per_count = 2*M_PI/resolution;
    }

    double calcAngle(){
        return enc*rads_per_count;
    }
private:
    /* data */
};
