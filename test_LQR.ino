#include <SimpleFOC.h>
#include <config.h>

// ============================================================
// OBJECT INSTANTIATION
// ============================================================
BLDCMotor motor = BLDCMotor(Pins::MOTOR_PP);
BLDCDriver3PWM driver = BLDCDriver3PWM(Pins::PWM_A, Pins::PWM_B, Pins::PWM_C, Pins::DRIVER_EN);
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, Pins::CURR_SENSE_A, Pins::CURR_SENSE_B);

Encoder motor_enc = Encoder(Pins::MOT_ENC_A, Pins::MOT_ENC_B, Config::ENC_CPR);
void doMotorA() { motor_enc.handleA(); }
void doMotorB() { motor_enc.handleB(); }

Encoder pendulum = Encoder(Pins::PEND_ENC_A, Pins::PEND_ENC_B, Config::ENC_CPR);
void doPendulumA() { pendulum.handleA(); }
void doPendulumB() { pendulum.handleB(); }

enum class ControlState { SWING_UP, STABILIZE, SAFETY_RESET };
ControlState currentState = ControlState::SWING_UP;
unsigned long lastControl = 0;

// ============================================================
// UTILS
// ============================================================
float wrapToPi(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

float applyMinimumTorque(float u) {
    float abs_u = abs(u);
    
    // 1. Noise floor: If it's effectively zero, keep it zero
    // This prevents the motor from humming when it's perfectly balanced
    if (abs_u < Config::TORQUE_ZERO_THRESHOLD) { 
        return 0.0f; 
    }
    
    // 2. Breakout: If it's between noise and movement threshold, snap to min
    if (abs_u < Config::MIN_ACTUATION_TORQUE) {
        // Return MIN_ACTUATION_TORQUE while preserving the original sign (+ or -)
        return (u > 0) ? Config::MIN_ACTUATION_TORQUE : -Config::MIN_ACTUATION_TORQUE;
    }
    
    // 3. Normal range: Just return the calculated value
    return u;
}

// ============================================================
// MODULAR CONTROL METHODS
// ============================================================

float runSwingUp(float p_angle, float p_vel) {
    const float K = 0.0008f;
    // Energy based swing-up logic
    float energy_term = 9.81f * (1.0f - cosf(p_angle)) - 0.0075f * p_vel * p_vel;
    float tau = K * powf(cosf(p_angle), 4) * p_vel * energy_term;
    return -1*_constrain(tau, -Config::TORQUE_LIMIT, Config::TORQUE_LIMIT);
}

float runLQR(float m_a, float m_v, float p_a, float p_v) {
    // Use effective_m_a for the position term calculation
    float u = ((Config::K1 * m_a) + (Config::K2 * m_v) + 
               (Config::K3 * p_a) + (Config::K4 * p_v)) * (1.0f / Config::RADIUS);
    float gain = 5;
    return -1*_constrain(u, -Config::TORQUE_LIMIT, Config::TORQUE_LIMIT)/gain;
}

float runRecovery(float m_a, float m_v) {
    constexpr float kP = 2.0f, kD = 0.1f;
    float u = (-kP * m_a) - (kD * m_v);
    return _constrain(u, -0.1f, 0.1f); // Gentler return to center
}

// DEBUG

unsigned long lastLogTime = 0;
const unsigned long logInterval = 200; // Print every 200ms (5 times per second)

// Create a global to hold the value for the loop to see
float target_torque_global = 0.0f;
float target_velocity_global = 0.0f;
float target_angle_pendulum_global = 0.0f;
float wrapped_angle_pendulum_global = 0.0f;

float current_motor_angle = 0.0f;
float current_motor_speed = 0.0f;
float current_pendulum_angle = 0.0f;
float current_pendulum_speed = 0.0f;


void debugPrint() {
    unsigned long now = millis();
    if (now - lastLogTime >= logInterval) {
        lastLogTime = now;

        const char* stateStr;
        switch (currentState) {
            case ControlState::SWING_UP:     stateStr = "SWNG"; break;
            case ControlState::STABILIZE:    stateStr = "STAB"; break;
            case ControlState::SAFETY_RESET: stateStr = "SAFE"; break;
        }

        char c1[8], c2[8], c3[8], c4[8], c5[8], c6[8], c7[8], c8[8], c9[8];
        dtostrf(current_motor_angle,                    6, 2, c1);
        dtostrf(wrapToPi(current_motor_angle),          6, 2, c2);
        dtostrf(target_angle_pendulum_global,           6, 2, c3);
        dtostrf(wrapped_angle_pendulum_global,          6, 2, c4);
        dtostrf(target_torque_global,                   6, 2, c5);
        dtostrf(applyMinimumTorque(target_torque_global), 6, 2, c6);
        dtostrf(current_motor_speed,                    6, 2, c7);
        dtostrf(current_pendulum_speed,                 6, 2, c8);
        dtostrf(target_velocity_global,                 6, 2, c9);

        char buf[120];
        snprintf(buf, sizeof(buf), "%s | %s | %s | %s | %s | %s | %s | %s | %s | %s",
            stateStr, c1, c2, c3, c4, c5, c6, c7, c8, c9);

        Serial.println(buf);
        Serial.flush();
    }
}

// ============================================================
// CORE LOGIC
// ============================================================


// Replace your controlStep() with this version:
void controlStep() {
    // 1. Force update both encoders
    motor_enc.update();
    pendulum.update();
    
    float p_angle = wrapToPi(pendulum.getAngle()+_PI);
    // float p_angle = wrapToPi(pendulum.getAngle());
    target_angle_pendulum_global = pendulum.getAngle();
    wrapped_angle_pendulum_global = p_angle;

    float p_vel   = pendulum.getVelocity();
    float m_angle = motor_enc.getAngle();
    float m_vel   = motor_enc.getVelocity();

    // float p_vel   = 0.0f;
    // float m_angle = 0.0f;
    // float m_vel   = 0.0f; 

    current_motor_speed = m_vel;
    current_pendulum_angle = p_angle;
    current_pendulum_speed = p_vel;
    current_motor_angle = m_angle;

    float effective_m_a = m_angle;
    float effective_m_v = m_vel;

    float effective_lqr_m_a = effective_m_a*1*Config::RADIUS;
    float effective_lqr_m_v = effective_m_v*1*Config::RADIUS;


    if (abs(m_angle) < Config::LQR_MOTOR_POS_DEADBAND) {
        effective_m_a = 0.0f;
        if (abs(m_vel) < Config::LQR_MOTOR_VEL_DEADBAND) {
            effective_m_v = 0.0f;
        }
    }

    float effective_p_a = p_angle;
    float effective_p_v = p_vel;

    if (abs(p_angle) < Config::LQR_PEND_POS_DEADBAND) {
        effective_p_a = 0.0f;
        if (abs(p_vel) < Config::LQR_PEND_VEL_DEADBAND) {
            effective_p_v = 0.0f;
        }
    }

    target_velocity_global = effective_m_v;

    // 1. Safety Check
    if (abs(m_angle) > Config::MOTOR_SOFT_LIMIT_RAD && currentState != ControlState::SAFETY_RESET) {
        currentState = ControlState::SAFETY_RESET;
    }


    // 2. State Transitions
    switch (currentState) {
        case ControlState::SAFETY_RESET:
            if (abs(m_angle) < Config::RESET_POS_THRESHOLD && abs(p_vel) < Config::RESET_VEL_THRESHOLD) 
                currentState = ControlState::SWING_UP;
            break;
            
        case ControlState::SWING_UP:
            if (abs(p_angle) < Config::STABILIZE_THRESHOLD) 
                currentState = ControlState::STABILIZE;
            break;
            
        case ControlState::STABILIZE:
            if (abs(p_angle) > Config::FALL_THRESHOLD) 
                currentState = ControlState::SWING_UP;
            break;
    }

    // 3. Action Execution
    float raw_target = 0;
    switch (currentState) {
        case ControlState::STABILIZE:    raw_target = runLQR(effective_lqr_m_a, effective_lqr_m_v, effective_p_a, effective_p_v); break;
        case ControlState::SWING_UP:     raw_target = runSwingUp(p_angle, p_vel);             break;
        case ControlState::SAFETY_RESET: raw_target = runRecovery(effective_m_a, effective_m_v);            break;
    }

    // Apply our minimum torque logic
    float final_target = applyMinimumTorque(raw_target);
    motor.move(final_target);
    target_torque_global = final_target;
}

void setup() {

    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for USB CDC to actually connect
    delay(500);
    Serial.println("BOOT OK");

    // In setup(), after BOOT OK:
    Serial.println("STATE   | MAng   | wMAng  | PAng   | wPAng  | Trq    | TrqTgt | MVel   | PVel   | effVel");
    Serial.println("--------|--------|--------|--------|--------|--------|--------|--------|--------|-------");
    Serial.flush();

    motor_enc.init();
    motor_enc.enableInterrupts(doMotorA, doMotorB);

    motor_enc.quadrature = Quadrature::ON;    
    motor_enc.pullup = Pullup::USE_EXTERN; 

    driver.voltage_power_supply = Config::VOLTAGE_LIMIT;
    driver.init();
    
    current_sense.linkDriver(&driver);
    current_sense.init();

    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::torque;

     // FOC current control parameters (PID gains)
    motor.PID_current_q.P = 5;
    motor.PID_current_q.I = 300;
    motor.PID_current_d.P = 5;
    motor.PID_current_d.I = 300;
    motor.LPF_current_q.Tf = 0.01; 
    motor.LPF_current_d.Tf = 0.01; 

    motor.linkSensor(&motor_enc);
    motor.linkDriver(&driver);
    motor.linkCurrentSense(&current_sense);

    motor.init();
    delay(100);
    motor.initFOC();

    delay(2000);
    pendulum.init();
    pendulum.enableInterrupts(doPendulumA, doPendulumB);
    pendulum.quadrature = Quadrature::ON;    
    pendulum.pullup = Pullup::USE_INTERN; 
}

void loop() {
    motor.loopFOC();
    unsigned long now = micros();
    if (now - lastControl >= Config::CONTROL_PERIOD_US) {
        lastControl = now;
        controlStep();
    }
    debugPrint();
}