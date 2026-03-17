#include "pid_controller.h"
#include <math.h>

// ================================================================
// PID Axis
// ================================================================

void pidAxisInit(PIDAxis *pid,
                 float kp, float ki, float kd,
                 float integralLimit, float outputLimit)
{
    pid->kp            = kp;
    pid->ki            = ki;
    pid->kd            = kd;
    pid->integralLimit = integralLimit;
    pid->outputLimit   = outputLimit;
    pidAxisReset(pid);
}

void pidAxisReset(PIDAxis *pid)
{
    pid->integral   = 0.0f;
    pid->lastError  = 0.0f;
    pid->lastOutput = 0.0f;
}

float pidAxisUpdate(PIDAxis *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // Integral with anti-windup clamp
    pid->integral += error * dt;
    pid->integral  = constrain(pid->integral,
                                -pid->integralLimit, pid->integralLimit);

    // Derivative on measurement to avoid "derivative kick" on setpoint step
    float derivative = (error - pid->lastError) / dt;
    pid->lastError   = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    output = constrain(output, -pid->outputLimit, pid->outputLimit);
    pid->lastOutput = output;
    return output;
}

// ================================================================
// Auto-Tune — relay-feedback (Ziegler-Nichols ultimate gain method)
// ================================================================
// The relay oscillates the system around zero at a fixed amplitude.
// After several cycles we measure the ultimate period Pu and the
// ultimate gain Ku, then apply the ZN formulas.
//
// Typical procedure:
//   1. Lock the drone on a test stand (prop wash only, no free flight).
//   2. Call autoTuneStart() for one axis.
//   3. Feed gyro error into autoTuneUpdate() instead of the normal PID.
//   4. After ~10 cycles (5–10 seconds) autoTuneDone() returns true.
//   5. Call autoTuneApply() to push gains into the PIDAxis.

#define AUTOTUNE_MIN_CYCLES  8

void autoTuneStart(PIDAutoTune *at, float relayAmp)
{
    at->active        = true;
    at->done          = false;
    at->relayAmplitude = relayAmp;
    at->peakPos       = 0.0f;
    at->peakNeg       = 0.0f;
    at->lastCrossing  = 0.0f;
    at->period        = 0.0f;
    at->cycleCount    = 0;
    at->resultKp = at->resultKi = at->resultKd = 0.0f;
}

float autoTuneUpdate(PIDAutoTune *at, float error, float dt)
{
    if (!at->active || at->done) return 0.0f;

    static float timeAcc = 0.0f;
    timeAcc += dt;

    // Track peaks
    if (error > at->peakPos) at->peakPos = error;
    if (error < at->peakNeg) at->peakNeg = error;

    // Relay output: bang-bang ±amplitude
    float output = (error >= 0.0f) ? at->relayAmplitude : -at->relayAmplitude;

    // Detect zero-crossings to measure oscillation period
    static float lastError = 0.0f;
    if (lastError < 0.0f && error >= 0.0f)   // rising crossing
    {
        if (at->lastCrossing > 0.0f)
        {
            float halfPeriod = timeAcc - at->lastCrossing;
            at->period  = 2.0f * halfPeriod;
            at->cycleCount++;

            if (at->cycleCount >= AUTOTUNE_MIN_CYCLES)
            {
                float amplitude = (at->peakPos - at->peakNeg) * 0.5f;
                if (amplitude > 1e-4f)
                {
                    // Ultimate gain Ku = 4d / (π * A)
                    float ku = (4.0f * at->relayAmplitude) / ((float)M_PI * amplitude);
                    float pu = at->period;

                    // Ziegler-Nichols PID tuning rules
                    at->resultKp = 0.60f * ku;
                    at->resultKi = 2.0f  * at->resultKp / pu;
                    at->resultKd = at->resultKp * pu / 8.0f;
                }
                at->active = false;
                at->done   = true;
            }
        }
        at->lastCrossing = timeAcc;
        at->peakPos = 0.0f;
        at->peakNeg = 0.0f;
    }
    lastError = error;
    return output;
}

bool autoTuneDone(PIDAutoTune *at) { return at->done; }

void autoTuneApply(PIDAutoTune *at, PIDAxis *pid)
{
    if (!at->done) return;
    pid->kp = at->resultKp;
    pid->ki = at->resultKi;
    pid->kd = at->resultKd;
    pidAxisReset(pid);
}
