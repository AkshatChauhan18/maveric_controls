#include "steering.h"
#include "driver/pulse_cnt.h"

// ============================================================
// Direction Conventions
// ============================================================

// Maps physical pin states to logical left/right steering directions
constexpr int DIR_THAT_DECREASES_COUNT = HIGH;
constexpr int DIR_LEFT  = DIR_THAT_DECREASES_COUNT;
constexpr int DIR_RIGHT = !DIR_THAT_DECREASES_COUNT;


// ============================================================
// PWM Settings
// ============================================================

constexpr int pwmFreq       = 5000;   // 5 kHz PWM frequency
constexpr int pwmResolution = 8;      // 8-bit resolution (0-255)


// ============================================================
// Limits & Deadband
// ============================================================

int minPWM   = 25;    // Minimum PWM to overcome static friction
int maxPWM   = 150;   // Maximum allowed PWM
int deadband = 10;     // Acceptable encoder error


// ============================================================
// PID Tuning Parameters
// ============================================================

float Kp = 0.20f;
float Ki = 0.001f;
float Kd = 0.02f;


// ============================================================
// Integral Limits
// ============================================================

float maxIntegral = 1000.0f;


// ============================================================
// State Variables
// ============================================================

volatile long targetPosition = 0;
bool pidEnabled              = false;
bool manualUnrestricted      = false;


// ============================================================
// Internal PID States
// ============================================================

static float lastError     = 0.0f;
static float integralError = 0.0f;
static unsigned long lastTime = 0;


// ============================================================
// Encoder Health Monitoring
// ============================================================

static bool encoderFault            = false;
static long lastEncoderCount        = 0;
static unsigned long lastMotionTime = 0;

// If motor is commanded but encoder doesn't move for this duration,
// trigger a safety fault.
constexpr unsigned long ENCODER_TIMEOUT_MS = 100;


// ============================================================
// NEW PCNT DRIVER
// ============================================================

// PCNT unit handle
static pcnt_unit_handle_t steeringPCNT = nullptr;

// Two PCNT channels for quadrature decoding
static pcnt_channel_handle_t channelA = nullptr;
static pcnt_channel_handle_t channelB = nullptr;


// ============================================================
// SETUP STEERING
// ============================================================

void setupSteering() {

    // Motor direction
    pinMode(PIN_DIR, OUTPUT);

    // Motor PWM
    ledcAttach(PIN_PWM, pwmFreq, pwmResolution);

    // Encoder inputs
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);

    // Initialize hardware quadrature encoder
    setupPCNT();
}


// ============================================================
// SETUP NEW PCNT DRIVER
// ============================================================

void setupPCNT() {

    // --------------------------------------------------------
    // Create PCNT unit
    // --------------------------------------------------------

    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767
    };

    ESP_ERROR_CHECK(
        pcnt_new_unit(
            &unit_config,
            &steeringPCNT
        )
    );


    // --------------------------------------------------------
    // Configure glitch filter
    // --------------------------------------------------------

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000
    };

    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(
            steeringPCNT,
            &filter_config
        )
    );


    // --------------------------------------------------------
    // CHANNEL A
    //
    // Edge signal  = Encoder A
    // Level signal = Encoder B
    // --------------------------------------------------------

    pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B
    };

    ESP_ERROR_CHECK(
        pcnt_new_channel(
            steeringPCNT,
            &channel_a_config,
            &channelA
        )
    );


    // Rising A  -> increase
    // Falling A -> decrease
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            channelA,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE
        )
    );


    // B HIGH -> keep
    // B LOW  -> inverse
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            channelA,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );


    // --------------------------------------------------------
    // CHANNEL B
    //
    // Edge signal  = Encoder B
    // Level signal = Encoder A
    // --------------------------------------------------------

    pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A
    };

    ESP_ERROR_CHECK(
        pcnt_new_channel(
            steeringPCNT,
            &channel_b_config,
            &channelB
        )
    );


    // Rising B  -> increase
    // Falling B -> decrease
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            channelB,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE
        )
    );


    // A HIGH -> inverse
    // A LOW  -> keep
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            channelB,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP
        )
    );


    // --------------------------------------------------------
    // Enable PCNT unit
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        pcnt_unit_enable(
            steeringPCNT
        )
    );


    // --------------------------------------------------------
    // Clear initial count
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        pcnt_unit_clear_count(
            steeringPCNT
        )
    );


    // --------------------------------------------------------
    // Start counting
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        pcnt_unit_start(
            steeringPCNT
        )
    );
}


// ============================================================
// GET ENCODER COUNT
// ============================================================

long getEncoderCount() {

    int count = 0;

    ESP_ERROR_CHECK(
        pcnt_unit_get_count(
            steeringPCNT,
            &count
        )
    );

    return (long)count;
}


// ============================================================
// ZERO ENCODER
// ============================================================

void zeroEncoderCount() {

    ESP_ERROR_CHECK(
        pcnt_unit_clear_count(
            steeringPCNT
        )
    );

    clearEncoderFault();
}


// ============================================================
// ENCODER FAULT STATUS
// ============================================================

bool isEncoderFault() {
    return encoderFault;
}


// ============================================================
// CLEAR ENCODER FAULT
// ============================================================

void clearEncoderFault() {

    encoderFault = false;

    lastMotionTime = millis();

    lastEncoderCount = getEncoderCount();
}


// ============================================================
// ENABLE PID
// ============================================================

void enablePID(long newTarget) {

    // Do not engage PID if encoder has a fault
    if (encoderFault) {
        return;
    }

    manualUnrestricted = false;

    // Clamp target to safe mechanical boundaries
    long clampedTarget =
        constrain(
            newTarget,
            MIN_POS_LIMIT,
            MAX_POS_LIMIT
        );


    // Reinitialize PID if target changed
    // or PID was previously disabled
    if (!pidEnabled || targetPosition != clampedTarget) {

        integralError = 0.0f;

        lastError =
            (float)(
                clampedTarget -
                getEncoderCount()
            );

        lastTime = millis();

        lastMotionTime = millis();

        lastEncoderCount = getEncoderCount();

        pidEnabled = true;
    }

    targetPosition = clampedTarget;
}


// ============================================================
// DISABLE PID
// ============================================================

void disablePID() {

    pidEnabled = false;

    integralError = 0.0f;

    ledcWrite(PIN_PWM, 0);
}


// ============================================================
// PID UPDATE
// ============================================================

void updatePID() {

    // Skip if PID disabled or encoder fault
    if (!pidEnabled || encoderFault) {
        return;
    }


    unsigned long now = millis();


    // Calculate delta time
    float dt =
        (now - lastTime) / 1000.0f;


    // Minimum update interval = 5 ms
    if (dt < 0.005f) {
        return;
    }

    lastTime = now;


    // --------------------------------------------------------
    // Read encoder
    // --------------------------------------------------------

    long currentPos = getEncoderCount();


    // Calculate position error
    float error =
        (float)(
            targetPosition -
            currentPos
        );


    // --------------------------------------------------------
    // Deadband
    // --------------------------------------------------------

    if (abs(error) <= deadband) {

        ledcWrite(PIN_PWM, 0);

        lastMotionTime = now;

        lastEncoderCount = currentPos;

        return;
    }


    // --------------------------------------------------------
    // Encoder Safety Check
    // --------------------------------------------------------

    if (currentPos != lastEncoderCount) {

        lastEncoderCount = currentPos;

        lastMotionTime = now;

    }
    else if (
        now - lastMotionTime >
        ENCODER_TIMEOUT_MS
    ) {

        encoderFault = true;

        stopSteeringMotor();

        Serial.println(
            "[SAFETY ALERT] Encoder not responding! "
            "Steering disabled."
        );

        return;
    }


    // --------------------------------------------------------
    // Integral
    // --------------------------------------------------------

    integralError += error * dt;

    integralError =
        constrain(
            integralError,
            -maxIntegral,
            maxIntegral
        );


    // --------------------------------------------------------
    // Derivative
    // --------------------------------------------------------

    float derivative =
        (error - lastError) / dt;

    lastError = error;


    // --------------------------------------------------------
    // PID Output
    // --------------------------------------------------------

    float output =
        (Kp * error) +
        (Ki * integralError) +
        (Kd * derivative);


    // --------------------------------------------------------
    // Direction
    // --------------------------------------------------------

    if (output > 0) {

        digitalWrite(
            PIN_DIR,
            DIR_RIGHT
        );

    }
    else {

        digitalWrite(
            PIN_DIR,
            DIR_LEFT
        );
    }


    // --------------------------------------------------------
    // PWM Magnitude
    // --------------------------------------------------------

    int pwmVal =
        (int)abs(output);


    // Minimum PWM to overcome friction
    if (pwmVal < minPWM) {
        pwmVal = minPWM;
    }


    // Maximum safe PWM
    pwmVal =
        constrain(
            pwmVal,
            0,
            maxPWM
        );


    // --------------------------------------------------------
    // Software Limit Safety
    // --------------------------------------------------------

    if (
        (currentPos >= MAX_POS_LIMIT && output > 0) ||
        (currentPos <= MIN_POS_LIMIT && output < 0)
    ) {

        ledcWrite(PIN_PWM, 0);

    }
    else {

        ledcWrite(
            PIN_PWM,
            pwmVal
        );
    }
}


// ============================================================
// MANUAL LIMIT CHECK
// ============================================================

void checkManualLimits() {

    // Don't interfere with PID
    // or unrestricted calibration
    if (
        pidEnabled ||
        manualUnrestricted
    ) {
        return;
    }


    long currentPos =
        getEncoderCount();


    int currentDir =
        digitalRead(PIN_DIR);


    // --------------------------------------------------------
    // Left limit
    // --------------------------------------------------------

    if (
        currentPos >= MAX_POS_LIMIT &&
        currentDir == DIR_RIGHT
    ) {

        ledcWrite(PIN_PWM, 0);
    }


    // --------------------------------------------------------
    // Right limit
    // --------------------------------------------------------

    else if (
        currentPos <= MIN_POS_LIMIT &&
        currentDir == DIR_LEFT
    ) {

        ledcWrite(PIN_PWM, 0);
    }
}


// ============================================================
// JOG LEFT
// ============================================================

void jogLeft(bool unrestricted) {

    disablePID();

    manualUnrestricted = unrestricted;


    // Move if unrestricted OR not at left limit
    if (
        unrestricted ||
        getEncoderCount() > MIN_POS_LIMIT
    ) {

        digitalWrite(
            PIN_DIR,
            DIR_LEFT
        );


        ledcWrite(
            PIN_PWM,
            minPWM + 30
        );


        Serial.println(
            unrestricted
                ? "Calibration LEFT..."
                : "Manual LEFT (Limit Protected)..."
        );
    }

    else {

        ledcWrite(
            PIN_PWM,
            0
        );


        Serial.println(
            "Manual LEFT blocked: "
            "At/Past MIN limit"
        );
    }
}


// ============================================================
// JOG RIGHT
// ============================================================

void jogRight(bool unrestricted) {

    disablePID();

    manualUnrestricted = unrestricted;


    // Move if unrestricted OR not at right limit
    if (
        unrestricted ||
        getEncoderCount() < MAX_POS_LIMIT
    ) {

        digitalWrite(
            PIN_DIR,
            DIR_RIGHT
        );


        ledcWrite(
            PIN_PWM,
            minPWM + 30
        );


        Serial.println(
            unrestricted
                ? "Calibration RIGHT..."
                : "Manual RIGHT (Limit Protected)..."
        );
    }

    else {

        ledcWrite(
            PIN_PWM,
            0
        );


        Serial.println(
            "Manual RIGHT blocked: "
            "At/Past MAX limit"
        );
    }
}


// ============================================================
// STOP STEERING MOTOR
// ============================================================

void stopSteeringMotor() {

    disablePID();

    manualUnrestricted = false;

    ledcWrite(
        PIN_PWM,
        0
    );
}