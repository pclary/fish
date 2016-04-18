#ifndef BLDCMOTOR_H
#define BLDCMOTOR_H


class BldcMotor
{
public:
    BldcMotor(int pin1, int pin2, int pin3, int enablePin = -1);
    BldcMotor(int pin1, int pin2, int pin3);

    void update(float fbDegrees);
    void setCurrent(float current);
    void setLeadAngle(float leadAngle);

    void setOutput(float magnitude, float angle);

    void enable();
    void disable();
    bool getEnabled();

private:
    int pin1, pin2, pin3, enablePin;

    float current = 0.f;
    float leadAngle = 60.f;

    bool enabled = true;
};

#endif // BLDCMOTOR_H
