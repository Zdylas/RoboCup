#include "MotorDriver.h"

MotorDriver::MotorDriver() {}

void MotorDriver::attach(int pin, bool is_forward) {
    motor.attach(pin);
    forward = is_forward;
}
    
void MotorDriver::setSpeed(float speed) {
    speed *= (forward == 0) ? 1 : -1;
    if (speed > 1.0) speed = 1.0;
    if (speed < -1.0) speed = -1.0;

    int pwm = 1500 + speed * 500;
    motor.writeMicroseconds(pwm);
}