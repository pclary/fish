#include "BldcMotor.h"
#include "Arduino.h"
#include <cmath>
#include "utilities.h"


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

void BldcMotor::update(float fbd)
{
    float degrees = fbd;
    if (current > 0.f)
        degrees += leadAngle;
    else if (current < 0.f)
        degrees -= leadAngle;

    setOutput(std::fabs(current), degrees);
}


void BldcMotor::setCurrent(float cur)
{
    current = cur;
}


void BldcMotor::setLeadAngle(float la)
{
    leadAngle = la;
}


void BldcMotor::setOutput(float magnitude, float degrees)
{
    const int maxPwmInt = 4095;

    float ph1 = degrees * pi/180.f;
    float ph2 = ph1 + pi*2.f/3.f;
    float ph3 = ph1 + pi*4.f/3.f;

    int pwmval1 = int((std::sin(ph1) + 1.f) / 2.f * maxPwmInt * magnitude);
    int pwmval2 = int((std::sin(ph2) + 1.f) / 2.f * maxPwmInt * magnitude);
    int pwmval3 = int((std::sin(ph3) + 1.f) / 2.f * maxPwmInt * magnitude);

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
