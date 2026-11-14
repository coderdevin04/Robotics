#ifndef USERAPI_H
#define USERAPI_H
#include <stdint.h>

// ============================================================================
//  ReRo User API -- the ONLY header participant code may include.
//  Your code implements:   void userLoop();   (called repeatedly, forever)
// ============================================================================

// ---- Timing ----
void waitMS(uint32_t ms);      // sleep without blocking the safety task
uint32_t getTimeMS();          // milliseconds since boot

// ---- Serial output (shown in the web terminal) ----
class UserSerial {
public:
    void print(const char* msg);
    void print(int value);
    void print(float value, int decimals = 2);
    void println(const char* msg = "");
    void println(int value);
    void println(float value, int decimals = 2);
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};
extern UserSerial userserial;

// ---- IR line sensor array ----
int  getIRSensorCount();              // number of sensors (left -> right)
void readIRArray(int* valuesOut);     // 1 = line under sensor, 0 = no line
void readIRRawArray(int* valuesOut);  // raw reflectance, 0 (white) to 2500 (black / no reflection)

// ---- Motors ----
#define FORWARD  1
#define BACKWARD 0
void motors_init();                               // optional; already done at boot
void setLeftSpeed(int direction, int value);      // value 0-255
void setRightSpeed(int direction, int value);     // value 0-255
void stopMotors();

// ---- Wheel encoders ----
void getEncoderCounts(int32_t* leftCount, int32_t* rightCount);
void resetEncoders();
void getWheelRotations(float* leftRot, float* rightRot);
void getWheelRPM(float* leftRPM, float* rightRPM);   // averaged since last call

// Internal: called once by the firmware before userLoop() starts.
void userapi_init();

// ============================================================================
//  Extras for this bot (ESP32-S3 + QTR-8RC + 2x Waveshare DDSM210).
//  Everything above is the RERO-26 platform contract, unchanged. The lines
//  below are additions; platform team code does not need them.
//  Speed 0-255 maps linearly to 0-70 rpm (safety cap). There are no encoders: rotations
//  and counts are integrated from the motors' rpm feedback (78.4 counts/rev).
//  Optional: define void userSetup() to run once before userLoop().
// ============================================================================
#include <stdio.h>   // snprintf
#include <math.h>    // fabsf, sinf, ...

uint32_t millisNow();                     // same as getTimeMS()
void calibrateIR(int samples = 100);      // sweep the sensor over line and floor while this runs
int  readLinePosition();                  // 0..7000 after calibrateIR(), 3500 = centred
void setWheelRPM(float leftRPM, float rightRPM);   // RPM, positive = forward, capped at +/-70
void setLeftRPM(float rpm);
void setRightRPM(float rpm);
void brakeMotors();                       // brake now (stopMotors() ramps down)
void getWheelRPMNow(float* leftRPM, float* rightRPM);  // latest motor feedback, not averaged
bool motorsOnline(bool* leftOk, bool* rightOk);     // true if both motors answer
bool readMotorMode(int* leftMode, int* rightMode);  // 2 = velocity mode, -1 = no reply

#endif
