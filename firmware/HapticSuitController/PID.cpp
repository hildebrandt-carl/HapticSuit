#include <math.h>
#include <Arduino.h>

// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umin, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umin(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float uminIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umin = uminIn, umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(float target, float deltaT, int &pwr, int &dir){
    // error
    int e = target;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }

    pwr = map(pwr, 0, umax, 0, umax-umin) + umin;
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }

    // store previous error
    eprev = e;

  }
  
};