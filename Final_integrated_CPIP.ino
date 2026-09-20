#include <Arduino.h>
#include <Wire.h>

// ===================== AS5600 =====================
#define AS5600_ADDR 0x36
#define RAW_ANGLE_HI 0x0C

const float OFFSET = 4.7571f;   // from LQRAItest
const float SCALE  = 0.998f;    // from LQRAItest

// ===================== ENCODER =====================
const int ENC_A = 2;
const int ENC_B = 3;

volatile long encoder_count = 0;
volatile uint8_t prev_state = 0;

void encoderISR() {
    uint8_t A = digitalReadFast(ENC_A);
    uint8_t B = digitalReadFast(ENC_B);
    uint8_t curr = (A << 1) | B;

    int8_t dir = 0;
    if ((prev_state == 0 && curr == 1) ||
        (prev_state == 1 && curr == 3) ||
        (prev_state == 3 && curr == 2) ||
        (prev_state == 2 && curr == 0)) dir = +1;
    else if ((prev_state == 0 && curr == 2) ||
             (prev_state == 2 && curr == 3) ||
             (prev_state == 3 && curr == 1) ||
             (prev_state == 1 && curr == 0)) dir = -1;

    encoder_count += dir;
    prev_state = curr;
}

// ===================== MOTOR =====================
const int PWM_FWD = 9;
const int PWM_REV = 10;
const int R_EN    = 22;
const int L_EN    = 23;

const float FMAX = 120.0f;

void setMotor(float u) {
    if (u >  FMAX) u =  FMAX;
    if (u < -FMAX) u = -FMAX;

    float u_norm = u / FMAX;
    int pwm = (int)(fabs(u_norm) * 1023.0f);
    if (pwm > 1023) pwm = 1023;

    if (u_norm >= 0) {
        analogWrite(PWM_FWD, pwm);
        analogWrite(PWM_REV, 0);
    } else {
        analogWrite(PWM_FWD, 0);
        analogWrite(PWM_REV, pwm);
    }
}

// ===================== CART POSITION =====================
const float PULLEY_DIAMETER = 0.0127f;
const float PULLEY_CIRC     = PI * PULLEY_DIAMETER;
const int   COUNTS_PER_REV  = 640;
const float X_PER_COUNT     = PULLEY_CIRC / COUNTS_PER_REV;

// ===================== LQR GAINS =====================
const float Kx      = 600.0f;
const float Kxdot   = 400.0f;
const float Ktheta  = 1250.0f;
const float Kthetad = 150.0f;
const float Ki_x    = 0.0f;

// ===================== UPRIGHT OFFSET =====================
const float THETA_UPRIGHT = 0.0089f;

// ===================== RAIL SAFETY =====================
const float RAIL_LIMIT = 0.50f;

// ===================== TIMING =====================
unsigned long lastMicros = 0;
const float dt_fixed = 0.001f;   // 1 ms

// FILTERED STATES
static float last_theta_f = 0.0f;
static float last_x       = 0.0f;

static float dx_f        = 0.0f;
static float theta_dot_f = 0.0f;
const float ALPHA        = 0.2f;
const float ANGLE_ALPHA  = 1.0f;

// ===================== MODE MACHINE =====================
enum Mode {
    SWING_UP,
    BALANCE,
    RETURN_TO_CENTER
};
Mode mode = SWING_UP;

// ===================== INTEGRAL TERM =====================
static float x_int = 0.0f;

// ===================== SPIN DETECTION (kept vars for restart logic) =====================
static float last_theta_bottom = 0.0f;
static int   wrap_count        = 0;
static unsigned long wrap_window_start = 0;
const unsigned long WRAP_WINDOW_US = 400000;

// ===================== SWING-UP CONTROL =====================
// *** TAPER REMOVED, k_e REMOVED ***
float swingUpControl(float theta_bottom, float theta_dot) {
    const float m = 0.25f;
    const float l = 0.35f;
    const float g = 9.81f;

    const float E_des = m * g * l;

    float E_kin = 0.5f * m * l * l * theta_dot * theta_dot;
    float E_pot = m * g * l * (1.0f - cosf(theta_bottom));
    float E     = E_kin + E_pot;

    float dE = E_des - E;

    const float k_ts = 25.0f;
    float F_ts = k_ts * sinf(theta_bottom);

    float F = F_ts;

    if (F >  FMAX) F =  FMAX;
    if (F < -FMAX) F = -FMAX;

    return F;
}

// ===================== RETURN-TO-CENTER CONTROL =====================
float returnToCenterControl(float x, float dx) {
    const float Kx_rc    = 300.0f;
    const float Kxdot_rc = 200.0f;

    float u_rc = -(Kx_rc * x + Kxdot_rc * dx);
    if (u_rc >  FMAX) u_rc =  FMAX;
    if (u_rc < -FMAX) u_rc = -FMAX;
    return u_rc;
}

// ===================== AS5600 READ =====================
uint16_t readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_HI);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);

    uint8_t high = Wire.read();
    uint8_t low  = Wire.read();
    return (high << 8) | low;
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    analogWriteResolution(10);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

    pinMode(PWM_FWD, OUTPUT);
    pinMode(PWM_REV, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    lastMicros = micros();

    Serial.println("Unified: Swing-Up + LQR + Return-to-Center");
}

// ===================== LOOP =====================
void loop() {
    while (micros() - lastMicros < 1000) { }
    lastMicros += 1000;
    float dt = dt_fixed;

    uint16_t raw = readRawAngle();
    float angle_raw = raw * (2.0f * PI / 4096.0f);

    float theta_sim = (angle_raw - OFFSET) * SCALE;
    if (theta_sim >  PI) theta_sim -= 2.0f * PI;
    if (theta_sim < -PI) theta_sim += 2.0f * PI;

    float theta_u = theta_sim - THETA_UPRIGHT;
    if (theta_u >  PI) theta_u -= 2.0f * PI;
    if (theta_u < -PI) theta_u += 2.0f * PI;

    float theta_bottom = theta_sim + PI;
    if (theta_bottom >  PI) theta_bottom -= 2.0f * PI;
    if (theta_bottom < -PI) theta_bottom += 2.0f * PI;

    static float theta_f_raw = 0.0f;
    theta_f_raw = theta_f_raw + ANGLE_ALPHA * (theta_sim - theta_f_raw);

    static float theta_f = 0.0f;
    theta_f = theta_f + ANGLE_ALPHA * (theta_u - theta_f);

    float x = encoder_count * X_PER_COUNT;

    float dx = (x - last_x) / dt;
    last_x = x;
    dx_f = dx_f + ALPHA * (dx - dx_f);

    float dtheta    = theta_f - last_theta_f;
    float theta_dot = dtheta / dt;
    last_theta_f    = theta_f;

    theta_dot_f = theta_dot_f + ALPHA * (theta_dot - theta_dot_f);

    static float last_theta_bottom_local = 0.0f;
    float dtheta_bottom = theta_bottom - last_theta_bottom_local;
    if (dtheta_bottom >  PI) dtheta_bottom -= 2.0f * PI;
    if (dtheta_bottom < -PI) dtheta_bottom += 2.0f * PI;
    float theta_dot_bottom = dtheta_bottom / dt;
    last_theta_bottom_local = theta_bottom;

    x_int += x * dt;

    float u_lqr =
        -(Kx * x +
          Ktheta * theta_f +
          Kxdot * dx_f +
          Kthetad * theta_dot_f +
          Ki_x * x_int);
    float u_balance = -u_lqr;

    float u_swing = swingUpControl(theta_bottom, theta_dot_bottom);

    float u_rc = returnToCenterControl(x, dx_f);

    // rail safety
    if (fabs(x) > RAIL_LIMIT) {
        mode = RETURN_TO_CENTER;
    }

    // spin detection REMOVED

    // fail out of balance if angle too large
    const float THETA_FAIL = 0.6f;
    if (mode == BALANCE && fabs(theta_f) > THETA_FAIL) {
        mode = RETURN_TO_CENTER;
    }

    // gate from swing-up to balance
    const float THETA_UP_GATE  = 0.35f;
    const float THETA_DOT_GATE = 1.5f;
    if (mode == SWING_UP &&
        fabs(theta_f) < THETA_UP_GATE &&
        fabs(theta_dot_f) < THETA_DOT_GATE) {
        mode = BALANCE;
    }

    // ===================== FIXED RESTART LOGIC =====================
    const float X_SMALL          = 0.07f;
    const float DX_SMALL         = 0.05f;
    const float THETA_BOTTOM_MAX = 1.2f;   // pole below ~70° from bottom

    if (mode == RETURN_TO_CENTER &&
        fabs(x) < X_SMALL &&
        fabs(dx_f) < DX_SMALL &&
        theta_bottom < THETA_BOTTOM_MAX) {

        mode = SWING_UP;
        wrap_count = 0;
        wrap_window_start = 0;
    }

    float u;

    switch (mode) {
        case SWING_UP:         u = u_swing;   break;
        case BALANCE:          u = u_balance; break;
        case RETURN_TO_CENTER: u = u_rc;      break;
        default:               u = 0.0f;      break;
    }

    static float u_slewed = 0.0f;
    float du = u - u_slewed;
    const float MAX_DU = 5.0f;
    if (du >  MAX_DU) du =  MAX_DU;
    if (du < -MAX_DU) du = -MAX_DU;
    u_slewed += du;

    setMotor(u_slewed);

    static int dbg = 0;
    if (++dbg >= 50) {
        dbg = 0;
        Serial.print(millis());        Serial.print(",");
        Serial.print(mode);           Serial.print(",");
        Serial.print(theta_sim*180.0f/PI);      Serial.print(",");
        Serial.print(theta_f*180.0f/PI);        Serial.print(",");
        Serial.print(theta_bottom*180.0f/PI);   Serial.print(",");
        Serial.print(theta_dot_f);              Serial.print(",");
        Serial.print(theta_dot_bottom);         Serial.print(",");
        Serial.print(x);                        Serial.print(",");
        Serial.print(dx_f);                     Serial.print(",");
        Serial.print(u_slewed);                 Serial.print(",");
        Serial.println(wrap_count);
    }
}

