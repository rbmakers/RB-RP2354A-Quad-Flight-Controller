#pragma once
#include <Arduino.h>

// ================================================================
// PID Controller
// ================================================================
// Three independent axis instances: roll, pitch, yaw.
// Each has its own Kp/Ki/Kd, integral limit, and output limit.
// Auto-tune uses a relay-feedback (Ziegler-Nichols) step-response
// method: ramp P until sustained oscillation, then back-calculate
// Ki and Kd.  Tune one axis at a time on a test stand.
// ================================================================

typedef struct
{
    float kp, ki, kd;
    float integralLimit;
    float outputLimit;
    // Internal state
    float integral;
    float lastError;
    float lastOutput;
} PIDAxis;

// Initialise one axis with gains and limits
void  pidAxisInit(PIDAxis *pid,
                  float kp, float ki, float kd,
                  float integralLimit, float outputLimit);

// Compute PID output.  dt in seconds.
float pidAxisUpdate(PIDAxis *pid, float setpoint, float measurement, float dt);

// Reset integrator and derivative state (use on mode-switch or disarm)
void  pidAxisReset(PIDAxis *pid);

// ---- Auto-tune (relay-feedback / Ziegler-Nichols) ---------------

typedef struct
{
    bool   active;
    bool   done;
    float  relayAmplitude;   // step size during oscillation test
    float  peakPos;
    float  peakNeg;
    float  lastCrossing;
    float  period;
    int    cycleCount;
    float  resultKp, resultKi, resultKd;
} PIDAutoTune;

void  autoTuneStart(PIDAutoTune *at, float relayAmp);
float autoTuneUpdate(PIDAutoTune *at, float error, float dt);
bool  autoTuneDone(PIDAutoTune *at);
void  autoTuneApply(PIDAutoTune *at, PIDAxis *pid);
