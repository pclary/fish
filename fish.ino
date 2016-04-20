#include <Arduino.h>
#include <IntervalTimer.h>
#include "BldcMotor.h"
#include "RingBuffer.h"


const int potAPin = A10;
const int potBPin = A11;
const int potAOffset = 2048;
const int potBOffset = 2048;

BldcMotor motorA(5, 3, 4, 0);
BldcMotor motorB(20, 6, 9, 1);
const float maxCurrent = 0.5f;

const float dt = 1.f/200.f;
IntervalTimer loopTimer;
void controlLoopFcn();

struct LogEntry
{
    float eangle;
    short pot;
};
RingBuffer<LogEntry, 6000> dataLog;
bool printing = false;


void setup()
{
    asm(".global _printf_float");

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);

    analogReadResolution(12);

    motorA.setOutput(maxCurrent, 0.f);
    motorA.enable();

    delay(100);
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
        snprintf(buf, 256, "%f,%d\n", dataLog[i].eangle, dataLog[i].pot);
        Serial.write(buf);
    }

    printing = false;
}


float eangle = 0.f;
float dir = 1.f;

void controlLoopFcn()
{
    if (eangle > 720.f)
        dir = -1.f;
    else if (eangle < -720.f)
        dir = 1.f;

    eangle += dir*1.f;

    motorA.setOutput(maxCurrent, eangle);

    if (!printing)
        dataLog.push({eangle, analogRead(potAPin)});
}
