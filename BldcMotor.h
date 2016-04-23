#ifndef BLDCMOTOR_H
#define BLDCMOTOR_H

#include <cstdint>

class BldcMotor
{
public:
    BldcMotor(int pin1, int pin2, int pin3, int enablePin = -1);
    BldcMotor(int pin1, int pin2, int pin3);

    void update(uint16_t fbangle);
    void setCurrent(int current);
    void setLeadAngle(uint16_t leadAngle);

    void setOutput(uint16_t magnitude, uint16_t angle);

    void enable();
    void disable();
    bool getEnabled();

private:
    int pin1, pin2, pin3, enablePin;

    int current = 0;
    uint16_t leadAngle = 10923u; // 60 deg

    bool enabled = true;
};

#endif // BLDCMOTOR_H
