#include <Arduino.h>
#include "BldcMotor.h"
#include "RingBuffer.h"
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

const int stepPin = 11;
const int dirPin = 12;
const int highLimPin = 7;
const int lowLimPin = 8;

LowPass potALp;
LowPass potBLp;
LowPass dpotALp;
LowPass dpotBLp;


void setup()
{
    asm(".global _printf_float");

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(921600);


    adc.setResolution(16, ADC_0);
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_0);
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_0);
    adc.setAveraging(16, ADC_0);
    adc.setResolution(16, ADC_1);
    adc.setConversionSpeed(ADC_HIGH_SPEED, ADC_1);
    adc.setSamplingSpeed(ADC_HIGH_SPEED, ADC_1);
    adc.setAveraging(16, ADC_1);
    adc.startSynchronizedContinuous(potBPin, potAPin);

    motorA.setOutput(0, 0);
    motorA.enable();
    motorB.setOutput(0, 0);
    motorB.enable();

    potALp.setCutoffFreq(200.f, dt);
    potBLp.setCutoffFreq(200.f, dt);

    dpotALp.setCutoffFreq(20.f, dt);
    dpotBLp.setCutoffFreq(20.f, dt);

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


const size_t command_length = 1 + 7 + 1;
uint8_t command_buffer[command_length];
unsigned int command_bytes_received = 0;


int stepperDir = HIGH;
int stepperSpeed = 0;
float refA = 0.f;
float refB = 0.f;
float drefA = 0.f;
float drefB = 0.f;

bool hlim = false;
bool llim = false;

uint8_t clast = 0;
bool decodeCommand();

void loop()
{
    if (Serial.available())
    {
        const uint8_t c = Serial.read();
        if (c == 0xff && clast == 0xff) // Sync with 0xff 0xff
            command_bytes_received = 0;

        command_buffer[command_bytes_received++] = c;
        clast = c;
    }

    if (command_bytes_received == command_length)
    {
        command_bytes_received = 0;

        // Response byte:
        // bit 0 (low): valid command received
        // bit 1:       high limit switch
        // bit 2:       low limit switch
        uint8_t response = 0;
        response |= decodeCommand() << 0;
        response |= hlim << 1;
        response |= llim << 2;
        Serial.write(response);
    }
}


bool decodeCommand()
{
    // Start byte
    if (command_buffer[0] != 0xff)
        return false;

    // 7-bit checksum
    const uint8_t chks = command_buffer[1] +
                         command_buffer[2] +
                         command_buffer[3] +
                         command_buffer[4] +
                         command_buffer[5] +
                         command_buffer[6] +
                         command_buffer[7];
    if ((chks & 0x7f) != command_buffer[8])
        return false;

    // Decode
    stepperSpeed = abs(int8_t(command_buffer[1]) * 31);
    stepperDir = int8_t(command_buffer[1]) > 0 ? HIGH : LOW;
    refA = (int16_t(command_buffer[2] << 8u) | uint8_t(command_buffer[3])) * 0.0234f;
    refB = (int16_t(command_buffer[4] << 8u) | uint8_t(command_buffer[5])) * 0.0234f;
    drefA = int8_t(command_buffer[6]) * 60.f;
    drefB = int8_t(command_buffer[7]) * 60.f;

    return true;
}


unsigned int stepPeriodCounter = 0;

float clamp(float f, float minmax)
{
    return f > minmax ? minmax : f < -minmax ? -minmax : f;
}


void controlLoopFcn()
{
    digitalWriteFast(13, HIGH);

    // Stepper direction
    digitalWriteFast(dirPin, stepperDir);

    // Feedback
    const auto adcVals = adc.readSynchronizedContinuous();
    const float eangleA = (int(uint16_t(adcVals.result_adc1)) + potAOffset)*pot2eangle;
    const float eangleB = (int(uint16_t(adcVals.result_adc0)) + potBOffset)*pot2eangle;

    const float potALast = potALp;
    const float potBLast = potBLp;
    potALp.push(eangleA);
    potBLp.push(eangleB);
    dpotALp.push((potALp - potALast)/dt);
    dpotBLp.push((potBLp - potBLast)/dt);

    // Controller
    const float errorA = refA - potALp;
    const float errorB = refB - potBLp;
    const float derrorA = drefA - dpotALp;
    const float derrorB = drefB - dpotBLp;

    const float kp = 0.01f;
    const float kd = 0.0002f;

    const float currentA = clamp(kp*errorA + kd*derrorA, maxCurrent);
    const float currentB = clamp(kp*errorB + kd*derrorB, maxCurrent);
    motorA.setCurrent(currentA * 65535.f);
    motorB.setCurrent(currentB * 65535.f);

    // Commutation
    motorA.update(int(potALp * 65536.f/360.f) % 65536);
    motorB.update(int(potBLp * 65536.f/360.f) % 65536);

    // Limit switches
    hlim = !digitalReadFast(highLimPin);
    llim = !digitalReadFast(lowLimPin);

    // Limit safeties
    if ( (hlim && stepperDir == HIGH) || (llim && stepperDir == LOW) )
        stepperSpeed = 0;

    // if (llim && hlim)
    //     stepperSpeed = 0;

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
