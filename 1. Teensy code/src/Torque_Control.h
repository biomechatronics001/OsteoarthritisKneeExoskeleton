#ifndef _TORQUE_CONTROL_H
#define _TORQUE_CONTROL_H
#include <Arduino.h>
//#include "Motor_Control_Pediatric_V2.h"
class Torque_Control
{
    public:
    double PID_out;
    Torque_Control();
    ~Torque_Control();
    void setPIDgain(double Pgain,double Igain,double Dgain);
    double Torque_PID_update(double Treference,double Tactual);
    void setMaximunPIDout(double maximum_value);      
    void setSampleFrequency(double sample_frequency);
    private:
    double P;
    double I;
    double D;
    double Tr;
    double Ta;
    double pre_error;
    double error;
    double sum_error;
    double sample_time;
    double maximum_out;
};
#endif
