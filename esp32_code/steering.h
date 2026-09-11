#ifndef STEERING_H
#define STEERING_H

#include <Arduino.h>

// --- Pin Definitions for Steering Control ---
constexpr int PIN_PWM   = 26;
constexpr int PIN_DIR   = 27;
constexpr int PIN_ENC_A = 14;
constexpr int PIN_ENC_B = 4;

// --- Mechanical & Drive Limits ---
constexpr long MIN_POS_LIMIT = -20000;
constexpr long MAX_POS_LIMIT =  20000;

// --- Global State Variables ---
extern volatile long targetPosition;
extern bool pidEnabled;
extern bool manualUnrestricted;

// --- Initialization Functions ---
void setupSteering();
void setupPCNT();

// --- Encoder Functions ---
long getEncoderCount();
void zeroEncoderCount();

// --- PID Controller Functions ---
void enablePID(long newTarget);
void disablePID();
void updatePID();
void checkManualLimits();

// --- Manual Control Functions ---
void jogLeft(bool unrestricted);
void jogRight(bool unrestricted);
void stopSteeringMotor();

// --- Safety & Encoder Diagnostics ---
bool isEncoderFault();
void clearEncoderFault();

#endif