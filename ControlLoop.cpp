#include "ControlLoop.h"


ControlLoop::ControlLoop(float dt) : 
    dt(dt), 
    kp(0.f), 
    ki(0.f), 
    kd(0.f), 
    ierror(0.f), 
    outputMax(1.f), 
    outputMin(-1.f),
    lastError(0.f)
{
}


void ControlLoop::setTuning(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}


void ControlLoop::setKp(float kp)
{
    this->kp = kp;
}


void ControlLoop::setKi(float ki)
{
    this->ki = ki;
}


void ControlLoop::setKd(float kd)
{
    this->kd = kd;
}


void ControlLoop::setDerivCutoffFreq(float freq)
{
    derror.setCutoffFreq(freq, dt);
}


void ControlLoop::setOutputLimits(float min, float max)
{
    outputMin = min;
    outputMax = max;
}


float ControlLoop::update(float error, float feedForward)
{
    float ierrorTemp = ierror + error * dt;
    
    derror.push((error - lastError) / dt);
    lastError = error;
    
    float control = feedForward + kp * error + ki * ierrorTemp + kd * derror;
    
    if (ki > 0.f)
    {
        // Don't integrate if the output is pinned at the limits
        if ( !(control > outputMax && error > 0.f) &&
             !(control < outputMin && error < 0.f) )
        {
            ierror = ierrorTemp;
        }
        
        // Limit the integral to the amount needed to saturate the output
        if (ierror * ki > outputMax)
            ierror = outputMax / ki;
        if (ierror * ki < outputMin)
            ierror = outputMin / ki;
    }
    
    return control > outputMax ? outputMax : (control < outputMin ? outputMin : control);
}
