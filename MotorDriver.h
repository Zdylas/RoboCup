#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include <Servo.h>

class MotorDriver {
public:
    MotorDriver();

    // attach motor to a pin
    void attach(int pin, bool is_forward);

    // set speed from -1.0 to 1.0
    void setSpeed(float speed);

private:
    Servo motor;
    bool forward;
};

#endif
