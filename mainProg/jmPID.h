/*
    jmPID.h - Library for Teensy PID
    Created by James L. Mussi, March 17, 2015.
*/
#ifndef jmPID_h
#define jmPID_h

//#include "Arduino.h"
#include <stdint.h>     //uint types

    class jmPID
    {
        public:
        jmPID();
        void kPID(double kP,double kI,double kD); // PID control 
        void kPI(double kP,double kI); // PI control only 
        void kPD(double kP,double kD); // PD control only
        void windUp(float settlePercent, uint32_t timeDelay); // set windUp delaying parameters. timeDelay is in microseconds
        int32_t run(int32_t feedbackVAL, int32_t refVAL); // run a PID cycle
        int32_t runPreserve(int32_t feedbackVAL, int32_t refVAL); // run a PID without right shift re-scaling to proper scale. Preserves precision from fixed point conversions
        void scaleN(double kN); // set derivative scaling constant
        void bitShift(uint8_t shift); // set bit shifting of float inputs converted to fixed points to preserve precision. default is 10
        
        void flush(); // resets wind-up
        void end(); // Reset wind-up values and clears starting time
        
        private:
        int32_t _pidCalc();
        int32_t _kP; // Proportional Constant
        int32_t _kI; // Integration Constant
        int32_t _kD; // Derivative Constant
        int32_t _kN; // Derivative Scaling Constant
        int32_t _kT; // time Scaling Constant

        int32_t _refVAL; // error reference value
        int32_t _feedbackVAL; // the feedback input variable
        int32_t _feedbackERR; // the error variable
        int32_t _feedbackERRprev; // the previous iteration error variable
        
        
        uint32_t _dt;   // time change since last iteration
        int32_t _pid_I; // Integration from last time-step

        uint8_t _shift; // default bit shift value
        
        bool _firstRun; // first PID run flag to delay I and D calculation for 1 iteration.
        float _settlePercent;       // percentage of target should be reached before integration begins.
        uint32_t _timeDelay;  // time until integration begins
        uint32_t _windUpStartTime;  // time integration begins
        uint32_t _pidTimeNOW; // time of current PID iteration
        uint32_t _pidTimePREV; // time of previous PID iteration

        
};

#endif
