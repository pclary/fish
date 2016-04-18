#include <Arduino.h>
#include <Encoder.h>
#include "BldcMotor.h"
#include <cmath>
#include "utilities.h"
#include "RingBuffer.h"
#include "ControlLoop.h"


BldcMotor motor(5, 3, 4, 0);
Encoder encoder(7, 8);
const float ppr = 4000;
const float cnt2deg = 360.f * 4.f / ppr;
const float maxCurrent = 0.4f;

const float dt = 1.f/200.f;
IntervalTimer loopTimer;

struct LogEntry
{
    float current;
    float freq;
    float pref;
    float vref;
    int counts;
};
RingBuffer<LogEntry, 2048> dataLog;
bool printing = false;

ControlLoop speedLoop(dt);
ControlLoop positionLoop(dt);


void controlLoopFcn();


void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);

    delay(200);
    digitalWrite(13, LOW);

    speedLoop.setOutputLimits(-maxCurrent, maxCurrent);
    speedLoop.setTuning(1.e-5f, 1.e-3f, 0);

    positionLoop.setOutputLimits(-1.e4f, 1.e4f);
    positionLoop.setTuning(1.f, 0.f, 1.e-3f);
    positionLoop.setDerivCutoffFreq(20.f);

    delay(1000);

    motor.enable();
    motor.setOutput(maxCurrent, 0.f);

    delay(1000);
    encoder.write(0);

    loopTimer.priority(144);
    loopTimer.begin(controlLoopFcn, dt*1.e6f);

    motor.setCurrent(maxCurrent);

    digitalWrite(13, HIGH);
}


void loop()
{
    while (!Serial.available());

    while (Serial.read() != -1);

    char buf[256];

    printing = true;

    for (int i = dataLog.size(); i > 0; --i)
    {
        snprintf(buf, 256, "%f,%f,%f,%f,%d\n", dataLog[i].current, dataLog[i].freq, dataLog[i].pref, dataLog[i].vref, dataLog[i].counts);
        Serial.write(buf);
    }

    printing = false;
}


float lastp = 0.f;

void controlLoopFcn()
{
    float t = millis() * 1.e-3f;

    // const float minFreq = 0.1f;
    // const float maxFreq = 10.f;
    // const float sweepTime = 20.f;
    // float freq = std::exp(std::log(minFreq) + (std::fmod(t, sweepTime) / sweepTime) * (std::log(maxFreq) - std::log(minFreq)));
    // float current = std::sin(freq*2.f*pi*t) * maxCurrent;

    //float freq = 1.f;
    //float current = (std::fmod(t, freq) > freq*0.5f ? -1.f : 1.f) * maxCurrent;
    float freq = 10.f;
    int counts = encoder.read();
    float p = counts * cnt2deg;
    //float pref = std::sin(2.f*pi*freq*t) * 180.f;
    //float vref = positionLoop.update(pref - p);
    float vref = std::fmod(t, freq) / freq * 15000.f;//std::sin(2.f*pi*freq*t) * 20000.f;
    float pref = 0.f;
    float v = (p - lastp) / dt;
    float ff = std::exp(vref*3.86e-4f)*1.64e-3f + 6.49e-2f;
    float current = speedLoop.update(vref - v, ff);
    lastp = p;

    motor.setCurrent(current);
    motor.update(encoder.read() * cnt2deg);

    if (!printing)
        dataLog.push({current, freq, pref, vref, counts});
}
