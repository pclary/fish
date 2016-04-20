#include <Arduino.h>
#include <IntervalTimer.h>
#include "BldcMotor.h"
#include "RingBuffer.h"
#include "PIDController.h"
#include "LowPass.h"
#include <cmath>
#include <ADC.h>


const int potAPin = A10;
const int potBPin = A11;
const int potAOffset = -2094;
const int potBOffset = -2048;
const float pot2eangle = 0.566f;
ADC adc;

BldcMotor motorA(5, 3, 4, 0);
BldcMotor motorB(20, 6, 9, 1);
const float maxCurrent = 0.7f;

const float dt = 1.f/1000.f;
IntervalTimer loopTimer;
void controlLoopFcn();

PIDController pidA(dt);

struct LogEntry
{
    float eangle;
    float pot;
};
RingBuffer<LogEntry, 5000> dataLog;
bool printing = false;


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

    motorA.setOutput(0, 0.f);
    motorA.enable();

    delay(500);
    Serial.write("\n");
    digitalWrite(13, LOW);

    loopTimer.priority(144);
    loopTimer.begin(controlLoopFcn, dt*1.e6f);
}


void loop()
{
    while (!Serial.available());

    while (Serial.read() != -1);

    char buf[256];

    printing = true;

    for (int i = dataLog.size(); i > 0; --i)
    {
        snprintf(buf, 256, "%f,%f\n", dataLog[i].eangle, dataLog[i].pot);
        Serial.write(buf);
    }

    printing = false;
}


float eangle = 0.f;
float dir = 1.f;

void controlLoopFcn()
{
    digitalWriteFast(13, HIGH);

    // Reference
    // const float refA = (millis()/500) % 2 == 1 ? 180.f : -180.f;
    const float refA = 600.f*std::sin(millis()*0.02f);

    // Feedback
    const auto adcVals = adc.readSynchronizedContinuous();
    const float eangleA = (adcVals.result_adc1 + potAOffset)*pot2eangle;
    const float eangleB = (adcVals.result_adc0 + potAOffset)*pot2eangle;

    // Controller
    const float currentA = pidA.update(refA - eangleA);
    motorA.setCurrent(currentA);

    // Commutation
    motorA.update(eangleA);

    if (!printing)
        dataLog.push({eangleA, currentA});

    digitalWriteFast(13, LOW);
}
