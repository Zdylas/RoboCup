#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);

    void begin();           // set up pins + attach interrupts
    long getCount();        // returns current encoder count
    void reset();           // resets count to 0

    // Called from ISR (public so global ISR functions can call it)
    void handleInterrupt();

private:
    uint8_t _pinA, _pinB;
    volatile long _count;

    static Encoder* encoders[2];     // registry of encoder pointers
    static uint8_t encoderCount;      // number of registered encoders

    static void isr0();
    static void isr1();

    int _index; // index of this encoder in the registry
    uint8_t _lastState;
};

#endif
