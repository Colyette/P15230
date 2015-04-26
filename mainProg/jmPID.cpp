/*
    jmPID.cpp - Library for Teensy PID
    Created by James L. Mussi, March 17, 2015.
*/
/**
 * \modified Alyssa Colyette
 * \brief changed timing source for program to use in Linux system
 */

//#include "Arduino.h"
#include "jmPID.h"
#include <time.h>


jmPID::jmPID()
{
    
    _kP = 0; // Proportional Constant
    _kI = 0; // Integration Constant
    _kD = 0; // Derivative Constant
    _kN = 1; // Derivative Scaling Constant
    _kT = 1; // time Scaling Constant
    
    _refVAL = 0; // error reference value
    _feedbackVAL = 0; // the feedback input variable
    _feedbackERR = 0; // the error variable
    _feedbackERRprev = 0; // the previous iteration error variable
    
    
    _dt = 0;   // time change since last iteration
    _pid_I = 0; // Integration from last time-step
    
    _shift = 10; // default bit shift value
    
    _firstRun = true; // first PID run flag to delay I and D calculation for 1 iteration.
    _settlePercent = 0;       // percentage of target should be reached before integration begins.
    _timeDelay = 0;  // time until integration begins
    _windUpStartTime = 0;  // time integration begins
    _pidTimeNOW = 0; // time of current PID iteration

    
}// END jmPID

// PID control
void jmPID::kPID(double kP, double kI, double kD)
{
    // shifting bits of floating inputs into usable fixed point numbers
    _kP = (1 << _shift)*kP;
    _kI = (1 << _shift)*kI;
    _kD = (1 << _shift)*kD;
} //END kPID()

// PI control only 
void jmPID::kPI(double kP, double kI) 
{
    // shifting bits of floating inputs into usable fixed point numbers
    _kP = (1 << _shift)*kP;
    _kI = (1 << _shift)*kI;
    _kD = 0;
} //END kPI

// PD control only
void jmPID::kPD(double kP, double kD)
{
    // shifting bits of floating inputs into usable fixed point numbers
    _kP = (1 << _shift)*kP;
    _kI = 0;
    _kD = (1 << _shift)*kD;
} //END kpD()

// set error scaling factor
void jmPID::scaleN(double kN) 
{
    // shifting bits of floating inputs into usable fixed point numbers
    _kN = (1 << _shift)*kN;
} // END scaleN()

void jmPID::windUp(float settlePercent, uint32_t timeDelay) // set windUp delaying parameters. timeDelay is in microseconds
{
    _settlePercent = (1 << 16) * settlePercent; // percentage of target should be reached before integration begins.
    _timeDelay = timeDelay;  // time until integration begins
}

// Reset wind-up values
void jmPID::flush() 
{
    _pid_I = 0; 
    
} // END flush()

// Reset wind-up values and clears starting time
void jmPID::end() 
{
    _pid_I = 0; 
    _dt = 0;
    //_pidLoss = 0;
    _firstRun = true;

} // END end()

// set bit shifting of float inputs converted to fixed points to preserve precision. default is 10
void jmPID::bitShift(uint8_t shift) 
{
    _shift=shift;   
} // END bitShift()

// run a PID cycle
int32_t jmPID::run(int32_t feedbackVAL, int32_t refVAL) 
{
    
    // making inputs private values
    _feedbackVAL = feedbackVAL; 
    _refVAL = refVAL;
    
    // right shift to proper scale
    int32_t output = _pidCalc() >> _shift;
    
    return output;
}// END run()
    
// run a PID without right shift re-scaling to proper scale. Perseveres precision from fixed point conversions
int32_t jmPID::runPreserve(int32_t feedbackVAL, int32_t refVAL) 
{
    
    // making inputs private values
    _feedbackVAL = feedbackVAL; 
    _refVAL = refVAL;   
    int32_t output = _pidCalc();
    
    return output;      
}// END runPreserve()

int32_t jmPID::_pidCalc()
{
    
    //LINUX specific
    long int start_time;
    struct timespec gettime_now;

    int32_t _pid_SUM = 0; // PID error sum junction
    int32_t _pid_OUT = 0; // final PID output
    int32_t _feedbackERR = 0;  // Error input to PID
    
    _feedbackERR = (_refVAL - _feedbackVAL);
        
    if (_kP) 
    {
        _pid_SUM +=  _kP * _feedbackERR; // proportional
    }
        

    if(_kI || _kD) // only get time if I or D is used
    {
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        _pidTimeNOW = (gettime_now.tv_nsec /1000);		//Get nS value
        //_pidTimeNOW = micros();
        _dt = _pidTimeNOW - _pidTimePREV;
        _pidTimePREV = _pidTimeNOW;
        
        if(_firstRun)
        {   // set flag to no longer the first run
            _firstRun = false;
            _windUpStartTime = _pidTimeNOW + _timeDelay;
        }
        else { // only execute I and D if not first iteration
            if (_kI && _pidTimeNOW >= _windUpStartTime) // integral , check wind-up time
            {
                _pid_I += _feedbackERR * _dt; // integration using forward Euler method
                _pid_SUM += _kI* _pid_I;
            }       
            if (_kD) // derivative
            {
                if (!_dt) 
                { // divide by zero incoming ABORT!!!
                    // Serial.println("PID DIV0 ABORTED!!!!");
                }
                else 
                {
                    _pid_SUM += _kD * (_kN*(_feedbackERR - _feedbackERRprev)) /(_dt);  // right hand derivative
                    _feedbackERRprev = _feedbackERR;
                }
            }
        }
    }
        
    _pid_OUT = _pid_SUM >> _shift; // rescale to final OutPut Value
        
    return _pid_OUT; 
} // END pidCalc()








