#include <Arduino.h>
#include "BldcMotor.h"
#include "RingBuffer.h"
#include "PIDController.h"
#include "LowPass.h"
#include <cmath>
#include <ADC.h>


const int potAPin = A10;
const int potBPin = A11;
const int potAOffset = -2094;
const int potBOffset = -2215;
const float pot2eangle = 0.568f;
ADC adc;

BldcMotor motorA(5, 3, 4, 0);
BldcMotor motorB(20, 6, 9, 1);
const float maxCurrent = 0.7f;

const float dt = 1.f/4000.f;
IntervalTimer loopTimer;
void controlLoopFcn();

PIDController pidA(dt);
PIDController pidB(dt);

const int stepPin = 11;
const int dirPin = 12;
const int highLimPin = 7;
const int lowLimPin = 8;


void setup()
{
    asm(".global _printf_float");

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);


    adc.setResolution(12, ADC_0);
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_0);
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_0);
    adc.setAveraging(16, ADC_0);
    adc.setResolution(12, ADC_1);
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_1);
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_1);
    adc.setAveraging(16, ADC_1);
    adc.startSynchronizedContinuous(potBPin, potAPin);


    pidA.setOutputLimits(-maxCurrent, maxCurrent);
    pidA.setTuning(0.01f, 0.f, 0.0001f);
    pidA.setDerivLowpassFreq(100.f);

    pidB.setOutputLimits(-maxCurrent, maxCurrent);
    pidB.setTuning(0.01f, 0.f, 0.0001f);
    pidB.setDerivLowpassFreq(100.f);

    motorA.setOutput(0, 0.f);
    motorA.enable();
    motorB.setOutput(0, 0.f);
    motorB.enable();

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
int stepPeriod = 10;
int stepPeriodCounter = 0;


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
    const float eangleA = (adcVals.result_adc1 + potAOffset)*pot2eangle;
    const float eangleB = (adcVals.result_adc0 + potBOffset)*pot2eangle;

    // Controller
    const float currentA = pidA.update(refA - eangleA);
    const float currentB = pidB.update(refB - eangleB);
    motorA.setCurrent(currentA);
    motorB.setCurrent(currentB);

    // Commutation
    motorA.update(eangleA);
    motorB.update(eangleB);

    // Limit switches
    const bool hlim = !digitalReadFast(highLimPin);
    const bool llim = !digitalReadFast(lowLimPin);

    if (hlim)
        stepperDir = LOW;

    if (llim)
        stepperDir = HIGH;

    // Step pulse
    if (stepPeriodCounter >= stepPeriod)
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
