#include <Arduino.h>
#include "BldcMotor.h"
#include "RingBuffer.h"
#include "PIDController.h"
#include "LowPass.h"
#include <cmath>
#include <ADC.h>


const int potAPin = A10;
const int potBPin = A11;
const int potAOffset = -2094*16;
const int potBOffset = -2215*16;
const float pot2eangle = 0.568f/16.f;
ADC adc;

BldcMotor motorA(5, 3, 4, 0);
BldcMotor motorB(20, 6, 9, 1);
const float maxCurrent = 0.7f;

const unsigned int frequency = 8000;
const float dt = 1.f/frequency;
IntervalTimer loopTimer;
void controlLoopFcn();

PIDController pidA(dt);
PIDController pidB(dt);

const int stepPin = 11;
const int dirPin = 12;
const int highLimPin = 7;
const int lowLimPin = 8;

LowPass potALp;
LowPass potBLp;


void setup()
{
    asm(".global _printf_float");

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);


    adc.setResolution(16, ADC_0);
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_0);
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_0);
    adc.setAveraging(16, ADC_0);
    adc.setResolution(16, ADC_1);
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_1);
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_1);
    adc.setAveraging(16, ADC_1);
    adc.startSynchronizedContinuous(potBPin, potAPin);


    pidA.setOutputLimits(-maxCurrent, maxCurrent);
    pidA.setTuning(0.01f, 0.f, 0.0001f);
    pidA.setDerivLowpassFreq(50.f);

    pidB.setOutputLimits(-maxCurrent, maxCurrent);
    pidB.setTuning(0.01f, 0.f, 0.0001f);
    pidB.setDerivLowpassFreq(50.f);

    motorA.setOutput(0, 0);
    motorA.enable();
    motorB.setOutput(0, 0);
    motorB.enable();

    potALp.setCutoffFreq(200.f, dt);
    potBLp.setCutoffFreq(200.f, dt);

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(highLimPin, INPUT_PULLUP);
    pinMode(lowLimPin, INPUT_PULLUP);

    delay(500);
    Serial.write("\n");
    digitalWrite(13, LOW);

    loopTimer.priority(144);
    loopTimer.begin(controlLoopFcn, dt*1.e6f);
}


void loop()
{
}


int stepperDir = HIGH;
int stepperSpeed = 0;
unsigned int stepPeriodCounter = 0;


void controlLoopFcn()
{
    digitalWriteFast(13, HIGH);

    // Stepper direction
    digitalWriteFast(dirPin, stepperDir);

    // Reference
    // const float refA = (millis()/500) % 2 == 1 ? 180.f : -180.f;
    const float refA = 360.f;
    const float refB = -600.f;

    // Feedback
    const auto adcVals = adc.readSynchronizedContinuous();
    const float eangleA = (int(uint16_t(adcVals.result_adc1)) + potAOffset)*pot2eangle;
    const float eangleB = (int(uint16_t(adcVals.result_adc0)) + potBOffset)*pot2eangle;
    potALp.push(eangleA);
    potBLp.push(eangleB);

    // Controller
    const float currentA = pidA.update(refA - potALp);
    const float currentB = pidB.update(refB - potBLp);
    motorA.setCurrent(currentA * 65535.f);
    motorB.setCurrent(currentB * 65535.f);

    // Commutation
    motorA.update(int(eangleA * 65536.f/360.f) % 65536);
    motorB.update(int(eangleB * 65536.f/360.f) % 65536);

    // Limit switches
    const bool hlim = !digitalReadFast(highLimPin);
    const bool llim = !digitalReadFast(lowLimPin);

    // if (hlim)
    // {
    //     if (stepperSpeed == 0)
    //         stepperDir = LOW;

    //     if (stepperDir == HIGH)
    //         stepperSpeed -= 20;
    //     else if (stepperSpeed < 4000)
    //         stepperSpeed += 30;
    // }

    // if (llim)
    // {
    //     if (stepperSpeed == 0)
    //         stepperDir = HIGH;

    //     if (stepperDir == LOW)
    //         stepperSpeed -= 20;
    //     else if (stepperSpeed < 4000)
    //         stepperSpeed += 30;
    // }

    if (hlim)
    {
        stepperSpeed = 8000;
        stepperDir = LOW;
    }
    else if (llim)
    {
        stepperSpeed = 8000;
        stepperDir = HIGH;
    }


    if (llim && hlim)
        stepperSpeed = 0;

    // Step pulse
    const unsigned int stepPeriod = frequency / stepperSpeed - 1;

    if (stepPeriodCounter >= stepPeriod && stepperSpeed != 0)
    {
        digitalWriteFast(stepPin, HIGH);
        stepPeriodCounter = 0;
        delayMicroseconds(2);
        digitalWriteFast(stepPin, LOW);
    }
    else
    {
        ++stepPeriodCounter;
    }

    digitalWriteFast(13, LOW);
}
