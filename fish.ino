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
const int startPin = 15;


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
    adc.setConversionSpeed(ADC_LOW_SPEED, ADC_0);
    adc.setSamplingSpeed(ADC_LOW_SPEED, ADC_0);
    adc.setAveraging(16, ADC_0);
    adc.setResolution(16, ADC_1);
    adc.setConversionSpeed(ADC_LOW_SPEED, ADC_1);
    adc.setSamplingSpeed(ADC_LOW_SPEED, ADC_1);
    adc.setAveraging(16, ADC_1);
    adc.startSynchronizedContinuous(potBPin, potAPin);

    motorA.setOutput(0, 0);
    motorA.enable();
    motorB.setOutput(0, 0);
    motorB.enable();

    potALp.setCutoffFreq(1000.f, dt);
    potBLp.setCutoffFreq(1000.f, dt);

    dpotALp.setCutoffFreq(100.f, dt);
    dpotBLp.setCutoffFreq(100.f, dt);

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(highLimPin, INPUT_PULLUP);
    pinMode(lowLimPin, INPUT_PULLUP);
    pinMode(startPin, INPUT_PULLUP);

    delay(500);
    Serial.write("\n");
    digitalWrite(13, LOW);

    loopTimer.priority(144);
    loopTimer.begin(controlLoopFcn, dt*1.e6f);
}


const size_t command_length = 1 + 10 + 1;
uint8_t command_buffer[command_length];
unsigned int command_bytes_received = 0;


bool homing = false;
int stepperSpeedMax = 0;
int stepperTarget = 0;
int stepperPosition = 0;
float refA = 0.f;
float refB = 0.f;
float drefA = 0.f;
float drefB = 0.f;

bool hlim = false;
bool llim = false;
bool start = false;

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
        response |= hlim  << 1;
        response |= llim  << 2;
        response |= start << 3;
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
                         command_buffer[7] +
                         command_buffer[8] +
                         command_buffer[9] +
                         command_buffer[10];
    if ((chks & 0x7f) != command_buffer[11])
        return false;

    // Decode
    stepperSpeedMax = abs(uint8_t(command_buffer[1]) * 16);
    stepperTarget = (uint16_t(command_buffer[2] << 8u) | uint8_t(command_buffer[3])) / 32;
    refA = (int16_t(command_buffer[4] << 8u) | uint8_t(command_buffer[5])) * 0.0234f;
    refB = (int16_t(command_buffer[6] << 8u) | uint8_t(command_buffer[7])) * 0.0234f;
    drefA = int8_t(command_buffer[8]) * 60.f;
    drefB = int8_t(command_buffer[9]) * 60.f;
    const uint8_t flags = command_buffer[10];
    homing = homing || (flags & 0x01);

    return true;
}


unsigned int stepPeriodCounter = 0;

float clamp(float f, float minmax)
{
    return f > minmax ? minmax : f < -minmax ? -minmax : f;
}


float deadband(float f, float deadband)
{
    return f > deadband ? f - deadband : f < -deadband ? f + deadband : 0.f;
}


void controlLoopFcn()
{
    digitalWriteFast(13, HIGH);

    // Calculate stepper speed and direction to hit target
    // If homing, slowly move in the negative direction
    int stepperDir;
    int stepperSpeed;
    if (homing)
    {
        stepperSpeed = 800;
        stepperDir = LOW;
    }
    else
    {
        if (stepperTarget > stepperPosition)
        {
            stepperSpeed = stepperSpeedMax;
            stepperDir = HIGH;
        }
        else if (stepperTarget < stepperPosition)
        {
            stepperSpeed = stepperSpeedMax;
            stepperDir = LOW;
        }
        else
        {
            stepperSpeed = 0;
            stepperDir = HIGH;
        }
    }

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
    const float errorA = deadband(refA - potALp, 0.f);
    const float errorB = deadband(refB - potBLp, 0.f);
    const float derrorA = deadband(drefA - dpotALp, 500.f);
    const float derrorB = deadband(drefB - dpotBLp, 500.f);

    const float kp = 0.01f;
    const float kd = 0.0001f;

    const float currentA = clamp(kp*errorA + kd*derrorA, maxCurrent);
    const float currentB = clamp(kp*errorB + kd*derrorB, maxCurrent);
    motorA.setCurrent(currentA * 65535.f);
    motorB.setCurrent(currentB * 65535.f);

    // Commutation
    motorA.update(int(potALp * 65536.f/360.f) % 65536);
    motorB.update(int(potBLp * 65536.f/360.f) % 65536);

    // Limit switches
    hlim  = !digitalReadFast(highLimPin);
    llim  = !digitalReadFast(lowLimPin);
    start = !digitalReadFast(startPin);

    // If homing, set the zero when the lower limit switch is pressed
    if (homing && llim)
    {
        stepperPosition = 0;
        homing = false;
    }

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
        if (stepperDir == HIGH)
            ++stepperPosition;
        else
            --stepperPosition;
        digitalWriteFast(stepPin, LOW);
    }
    else
    {
        ++stepPeriodCounter;
    }

    digitalWriteFast(13, LOW);
}
