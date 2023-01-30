#include "Torque_Control.h"

Torque_Control::Torque_Control()
{
    P=0;
    I=0;
    D=0;
    Tr=0;
    Ta=0;
    pre_error=0;
    error=0;
    sum_error=0;
    PID_out=0;
    sample_time=0.002; // !!!
    maximum_out=10;
}
Torque_Control::~Torque_Control()
{}
double Torque_Control::Torque_PID_update(double Treference,double Tactual)
{
  Tr=Treference;
  Ta=Tactual;
  error = Tr - Ta;
  sum_error=sum_error+error;
  
  if (P != 0) {
    PID_out = P * error;
  }
  else
  {
    PID_out=0;
  }
  if (I != 0) {
    PID_out = PID_out + I * (sum_error) * sample_time;
  }
  if (D != 0) {
    PID_out = PID_out + D * (error - pre_error) / sample_time;
  }
  pre_error = error;
  if (PID_out > maximum_out)
  {
    PID_out = maximum_out;
  }
  else if (PID_out < -maximum_out)
  {
    PID_out = -maximum_out;
  }
  return PID_out;
}
void Torque_Control::setPIDgain(double Pgain,double Igain,double Dgain)
{
  P=Pgain;
  I=Igain;
  D=Dgain;
}
void Torque_Control::setMaximunPIDout(double maximum_value)
{
  maximum_out=maximum_value;
}
void Torque_Control::setSampleFrequency(double sample_frequency)
{
  sample_time=1/sample_frequency;
}
