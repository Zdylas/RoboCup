#include "Encoder.h"

// static members
Encoder* Encoder::encoders[2] = { nullptr };
uint8_t Encoder::encoderCount = 0;

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : _pinA(pinA), _pinB(pinB), _count(0) {
    _index = encoderCount++;
    encoders[_index] = this;    

    _lastState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
}


void Encoder::isr0() { 
    if (encoders[0]) encoders[0]->handleInterrupt(); 
}

void Encoder::isr1() { 
    if (encoders[1]) encoders[1]->handleInterrupt(); 
}

void Encoder::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    if (_index == 0) {
        attachInterrupt(digitalPinToInterrupt(_pinA), isr0, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_pinB), isr0, CHANGE);
    } else if (_index == 1) {
        attachInterrupt(digitalPinToInterrupt(_pinA), isr1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_pinB), isr1, CHANGE);
    }
}

long Encoder::getCount() {
    noInterrupts();
    long val = _count;
    interrupts();
    return val;
}

void Encoder::reset() {
    noInterrupts();
    _count = 0;
    interrupts();
}

void Encoder::handleInterrupt() {
    uint8_t state = (digitalReadFast(_pinA) << 1) | digitalReadFast(_pinB);

    // Gray code sequence: 00 -> 01 -> 11 -> 10 -> 00 (CW), opposite for CCW
    if ((_lastState == 0b00 && state == 0b01) ||
        (_lastState == 0b01 && state == 0b11) ||
        (_lastState == 0b11 && state == 0b10) ||
        (_lastState == 0b10 && state == 0b00)) {
        _count++;
    } 
    else if ((_lastState == 0b00 && state == 0b10) ||
             (_lastState == 0b10 && state == 0b11) ||
             (_lastState == 0b11 && state == 0b01) ||
             (_lastState == 0b01 && state == 0b00)) {
        _count--;
    }

    _lastState = state;
}

