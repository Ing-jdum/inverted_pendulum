#pragma once
#include <Arduino.h>

namespace Pins {
    constexpr int MOTOR_PP      = 11;
    constexpr int PWM_A         = 6;
    constexpr int PWM_B         = 10;
    constexpr int PWM_C         = 5;
    constexpr int DRIVER_EN     = 8;
    
    constexpr int MOT_ENC_A     = 3;
    constexpr int MOT_ENC_B     = 2;
    
    constexpr int PEND_ENC_A    = A4;
    constexpr int PEND_ENC_B    = A5;
    
    constexpr int CURR_SENSE_A  = A0;
    constexpr int CURR_SENSE_B  = A2;
}

// ============================================================
// PHYSICS & CONTROL CONSTANTS
// ============================================================
namespace Config {
    // Encoder
    constexpr int ENC_CPR       = 1024;
    
    // Limits
    constexpr float VOLTAGE_LIMIT = 12.0f;
    constexpr float TORQUE_LIMIT  = 2.0f;
    constexpr float MOTOR_SOFT_LIMIT_RAD = 3.0f * _PI;
    constexpr float LQR_MOTOR_DEADBAND = 0.1f; 
    constexpr float MIN_ACTUATION_TORQUE =  0.1f;
    constexpr float TORQUE_ZERO_THRESHOLD = 0.02f;
    constexpr float LQR_MOTOR_POS_DEADBAND = 0.1f;  // rad
    constexpr float LQR_MOTOR_VEL_DEADBAND = 0.2f;  // rad/s
    constexpr float LQR_PEND_POS_DEADBAND = 0.01f;  // rad
    constexpr float LQR_PEND_VEL_DEADBAND = 2.0f;   // rad/s
    
    // Transitions (Hysteresis prevents "chattering" between states)
    constexpr float STABILIZE_THRESHOLD = 0.25f; // rad
    constexpr float FALL_THRESHOLD      = 0.25f; // rad
    constexpr float RESET_POS_THRESHOLD = 0.05f; // rad
    constexpr float RESET_VEL_THRESHOLD = 0.10f; // rad/s
    
    // LQR Gains (K = [pos_m, vel_m, pos_p, vel_p])
    constexpr float K1 = -0.8165f, K2 = -0.66789f, K3 = 3.8111f, K4 = 0.14243f;
    constexpr float RADIUS = 0.09;
    
    // Timing
    constexpr unsigned long CONTROL_PERIOD_US = 1000; // 1kHz
}