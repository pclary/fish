#ifndef GLOBALS_H
#define GLOBALS_H

#include "Wheel.h"
#include "vl6180x.h"
#include "LowPass.h"
#include "PIDController.h"
#include "BFS.h"
#include "Maze.h"
#include "Button.h"


const int led1Pin = 3;
const int led2Pin = 4;
const int buzzerPin = 5;
const int nfaultPin = 7;
const int encoderL1 = 14; // left encoder
const int encoderL2 = 15;
const int encoderR2 = 16; // right encoder
const int encoderR1 = 17;
const int motorRR = 20; // right reverse
const int motorRF = 21; // right forward motor pin
const int motorLR = 22; // left forward
const int motorLF = 23; // left reverse

const unsigned int controlFreq = 200; // Hz
const unsigned int controlPeriodUs = 1000000 / controlFreq;
const float dt = 1.f / controlFreq;

const float wheelCirc = 0.1f;
const int ppr = 1200;
const float count2dist = wheelCirc / ppr;
const float wheelBaseMin = 0.0805f;
const float wbConstant = 0.04f;
const float pi = 3.14159265f;
const float cellw = 0.18f;
const float wallw = 0.012f;
const float sensw = 0.067f;
const float fsensoff = 0.054f;

const int mazem = 16;
const int mazen = 16;

const unsigned int sensorFreq = 50; // Hz
const unsigned int sensorPeriodUs = 1000000 / sensorFreq;
const float dtsensor = 1.f / sensorFreq;

struct State
{
    float x;
    float y;
    float theta;
};

enum class Direction
{
    undefined,
    ipos,
    jpos,
    ineg,
    jneg
};

extern State state;
extern State target;
extern Node currentCell;
extern Node targetCell;
extern Node prevTargetCell;
extern int lCount;
extern int rCount;
extern LowPass drdt;
extern LowPass dfdt;
extern LowPass dldt;
extern LowPass dthdt;
extern LowPass dsdt;
extern float thgoal;
extern float therr;
extern float thctrl;
extern float speed;
extern float maxSpeed;
extern float ctheta;
extern float cside;
extern float cfront;
extern float targetDist;
extern Direction direction;
extern bool inDeadband;
extern Maze<16, 16> maze;
extern bool manualSlow;
extern float thetakp;
extern float thetakd;
extern float thctrlmax;
extern float wheelBase;

extern Button switch1;
extern Button switch2;
extern Button button1;
extern Button button2;

extern Wheel leftWheel;
extern Wheel rightWheel;

extern VL6180X rightSensor;
extern VL6180X frontSensor;
extern VL6180X leftSensor;

void rled(bool on);
void lled(bool on);


#endif // GLOBALS_H
