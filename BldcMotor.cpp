#include "BldcMotor.h"
#include "Arduino.h"
#include <cmath>

#include "lut.h"


BldcMotor::BldcMotor(int pin1, int pin2, int pin3, int enablePin) : pin1(pin1), pin2(pin2), pin3(pin3), enablePin(enablePin)
{
    const int pwmFrequency = 20000;

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);

    if (enablePin >= 0)
    {
        pinMode(enablePin, OUTPUT);
        enabled = false;
        digitalWrite(enablePin, enabled);
    }

    analogWriteFrequency(pin1, pwmFrequency);
    analogWriteFrequency(pin2, pwmFrequency);
    analogWriteFrequency(pin3, pwmFrequency);

    analogWriteResolution(12);
}

void BldcMotor::update(uint16_t fbangle)
{
    uint16_t currentangle = fbangle;
    if (current > 0)
        currentangle += leadAngle;
    else if (current < 0)
        currentangle -= leadAngle;

    const int abscurrent = std::abs(current);
    setOutput(min(abscurrent, 65535), currentangle);
}


void BldcMotor::setCurrent(int cur)
{
    current = cur;
}


void BldcMotor::setLeadAngle(uint16_t la)
{
    leadAngle = la;
}


void BldcMotor::setOutput(uint16_t magnitude, uint16_t angle)
{
    // Input angle has 16 bit range, but internally 0..1023 is used for 0-360 deg
    const size_t ph1 = angle / 64u;
    const size_t ph2 = (ph1 + 341u) % 1024u;
    const size_t ph3 = (ph1 + 693u) % 1024u;

    // Not even bothering to deduplicate portions of sine wave

    // Input magnitude has full 16 bit range
    // lookup table has a premultiplier of 4
    const uint32_t pwmval1 = (lut[ph1] * magnitude) / 1048576u;
    const uint32_t pwmval2 = (lut[ph2] * magnitude) / 1048576u;
    const uint32_t pwmval3 = (lut[ph3] * magnitude) / 1048576u;

    // Serial.print(magnitude);
    // Serial.print(", ");
    // Serial.print(angle);
    // Serial.print(", ");
    // Serial.print(ph1);
    // Serial.print(", ");
    // Serial.print(pwmval1);
    // Serial.print("\n");

    analogWrite(pin1, pwmval1);
    analogWrite(pin2, pwmval2);
    analogWrite(pin3, pwmval3);
}


void BldcMotor::enable()
{
    if (enablePin >= 0)
    {
        enabled = true;
        digitalWrite(enablePin, enabled);
    }
}


void BldcMotor::disable()
{
    if (enablePin >= 0)
    {
        enabled = false;
        digitalWrite(enablePin, enabled);
    }
}


bool BldcMotor::getEnabled()
{
    return enabled;
}


