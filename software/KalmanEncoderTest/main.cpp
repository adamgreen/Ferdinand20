/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Test for using LX-16A potentiometer readings as encoder.
#include <ctype.h>
#include <LX16A.h>
#include <Matrix2x2.h>


static float constrainAngle(float angle)
{
    // UNDONE: Need to handle more than +/- 2pi?
    if (angle < -(float)M_PI)
        return angle + 2.0f*(float)M_PI;
    else if (angle > (float)M_PI)
        return angle - 2.0f*(float)M_PI;
    else
        return angle;
}

class EncoderState
{
public:
    EncoderState(const Vector2& v)
    {
        set(v);
    }
    EncoderState()
    {
        m_angle = 0.0f;
        m_velocity = 0.0f;
    }

    void set(const Vector2& v)
    {
        m_vec = v;
    }

    void set(float angle, float velocity)
    {
        m_angle = angle;
        m_velocity = velocity;
    }

    void constrainAngle()
    {
        m_angle = ::constrainAngle(m_angle);
    }

    operator Vector2&()
    {
        return m_vec;
    }

    union
    {
        struct
        {
            float m_angle;
            float m_velocity;
        };
        Vector2 m_vec;
    };
};

static Timer          g_timer;
static LX16A_ServoBus g_servoBus(p9, p10);
static LX16A_Servo    g_servo(g_servoBus, 1);
volatile char         g_cmd = '\0';
#if !MRI_ENABLE
static Serial         g_serial(USBTX, USBRX);
#endif

static uint32_t       g_lastSampleTime;
static EncoderState   g_currState;
static Matrix2x2      g_kalmanP;
static Matrix2x2      g_kalmanQ;

static void displayCurrentAngle();
static void resetFilter();
static void runFilter();
static bool isServoInDeadZone(int16_t servoPos);
static void startStop();
static void nothing();
#if !MRI_ENABLE
static void serialReceiveISR();
#endif

static void (*g_pProcessRoutine)(void) = nothing;



int main()
{
#if !MRI_ENABLE
    g_serial.baud(230400);
    g_serial.attach(serialReceiveISR);
#endif // !MRI_ENABLE

    uint8_t servoId = g_servoBus.discoverServoId();
    printf("Servo ID = %u\n", servoId);
    g_servo.setServoId(servoId);
    g_servo.setRotationSpeed(0);
    g_servo.disableMotor();
    g_timer.start();

    while (true)
    {
        switch (tolower(g_cmd))
        {
            case 'i':
                printf("Idling...\n");
                g_servo.setRotationSpeed(0);
                g_servo.disableMotor();
                g_pProcessRoutine = nothing;
                g_cmd = '\0';
                break;
            case 'f':
                printf("Forward...\n");
                g_servo.setRotationSpeed(250);
                resetFilter();
                g_pProcessRoutine = runFilter;
                g_cmd = '\0';
                break;
            case 'r':
                printf("Reverse...\n");
                g_servo.setRotationSpeed(-500);
                resetFilter();
                g_pProcessRoutine = runFilter;
                g_cmd = '\0';
                break;
            case 's':
                printf("Start and stop...\n");
                g_servo.setRotationSpeed(1000);
                g_timer.reset();
                g_pProcessRoutine = startStop;
                g_cmd = '\0';
                break;
            case '\0':
                break;
            default:
                printf("'%c' is an unknown command.\n", g_cmd);
                g_pProcessRoutine = nothing;
                g_cmd = '\0';
            break;
        }

        g_pProcessRoutine();
    }
}



static void displayCurrentAngle()
{
    int16_t position = g_servo.getPosition();
    printf("%d,%.2f\n", position, position * LX16A_SERVO_TO_DEGREE_VALUE);
}

static void resetFilter()
{
    g_timer.reset();
    g_lastSampleTime = 0;
    g_currState.set(constrainAngle(LX16A_SERVO_TO_RADIAN_VALUE * g_servo.getPosition()), 0.0f);
    // UNDONE: I am not sure about these values, especially the last one.
    // Model error is caused when the velocity isn't constant as described by model.
    g_kalmanP.m_00 = 0.0001f; // UNDONE: 0.01f;
    g_kalmanP.m_01 = 0.0f;
    g_kalmanP.m_10 = 0.0f;
    g_kalmanP.m_11 = 0.001f; // UNDONE: 0.01f;

}

static void runFilter()
{
    // Calculate time since last running of the Kalman filter and grab the current servo position in close temporal
    // relation to the time capture.
    uint32_t currSampleTime = g_timer.read_us();
    int16_t servoPos = g_servo.getPosition();
    uint32_t elapsedMicroSec = currSampleTime - g_lastSampleTime;
    g_lastSampleTime = currSampleTime;
    float dt = (float)elapsedMicroSec / 1000000.0f;

    // Use a stddev of 1 lsb when not in dead zone.
    float kalmanR = 0.00001f; // UNDONE: LX16A_SERVO_TO_RADIAN_VALUE * LX16A_SERVO_TO_RADIAN_VALUE;
    // The servo position could be off by as much as +/-20 degrees when in dead zone of the pot.
    if (isServoInDeadZone(servoPos))
    {
        // +/-20 deg / 6 stddev_for_full_error = 40 / 6 = 6.7 degrees.
        kalmanR = 0.001f; // UNDONE: (LX16A_DEGREE_TO_RADIAN * 6.7f)*(LX16A_DEGREE_TO_RADIAN * 6.7f);
    }

    // Convert measured servo angle to be between -π and π radians.
    float z = constrainAngle(LX16A_SERVO_TO_RADIAN_VALUE * servoPos);

#ifdef UNDONE
    // Set Kalman Q covariance matrix, the model error, based on maximum acceleration and dt.
    const float maxAcceleration = 8470.0f;
    float maxVelocityDelta = maxAcceleration * dt;
    // 3 stdev to cover all of positive going error.
    float velocityStdDev = maxVelocityDelta / 3.0f;
    float velocityVariance = velocityStdDev * velocityStdDev;
    float positionStdDev = velocityStdDev * dt;
    float positionVariance = positionStdDev * positionStdDev;
    Matrix2x2 kalmanQ(positionVariance, 0.0f,
                      0.0f, velocityVariance);
#endif // UNDONE
    Matrix2x2 kalmanQ(0.00001f, 0.0f,
                      0.0f, 0.0005f);

    // A matrix for the system model.
    //   A = | 1 dt |  => pos(k+1) = pos(k) + vel(k)*dt
    //       | 0 1  |  => vel(k+1) = vel(k)
    Matrix2x2 A(1.0f, dt,
                0.0f, 1.0f);

    // Calculate Kalman prediction for x:
    //   xPredicted = A * prevXEstimate
    EncoderState xPredicted = EncoderState(A.multiply(g_currState));
    xPredicted.constrainAngle();

    // Calculate Kalman prediction for error:
    //   PPredicted = A * prevPEstimate * Atranspose + Q
    Matrix2x2 ap = A.multiply(g_kalmanP);
    Matrix2x2 apa = ap.multiplyTransposed(A);
    Matrix2x2 PPredicted = apa.addDiagonal(kalmanQ);

    // Calculate the Kalman gain.
    //   H = [1 0]
    //   K = PPredicted*Htranspose(H*PPredicted*Htranspose + R)^-1
    float hph = PPredicted.m_00;
    float sum = hph + kalmanR;
    Vector2 ph = Vector2(PPredicted.m_00, PPredicted.m_10);
    Vector2 K = ph.divide(sum);

    // Calculate the Kalman state estimate.
    //   H = [1 0]
    //   xEstimate = xPredicted + K*(z - H*xPredicted)
    float hp = xPredicted.m_angle;
    float diff = constrainAngle(fmodf(z - hp, 2.0f*(float)M_PI));
    Vector2 correction = K.multiply(diff);
    g_currState.set(((Vector2)xPredicted).add(correction));
    g_currState.constrainAngle();

    // Calculate the Kalman covariance (P) estimate.
    //   P = PPredicted - K*H*PPredicted
    float kh = K.m_0;
    Matrix2x2 correction2 = PPredicted.multiply(kh);
    g_kalmanP = PPredicted.subtract(correction2);

    printf("%lu,%f,%.2E,%.2E,%.2E,%.2E,%.2E,%.2E,%.2E,%.2E,%.2E,%.2E,%.2E,%f,%f,%f\n",
        elapsedMicroSec,
        z*LX16A_RADIAN_TO_DEGREE,
//        kalmanQ.m_00, kalmanQ.m_01, kalmanQ.m_10, kalmanQ.m_11,
        PPredicted.m_00, PPredicted.m_01, PPredicted.m_10, PPredicted.m_11,
        kalmanR,
        K.m_0, K.m_1,
        g_kalmanP.m_00, g_kalmanP.m_01, g_kalmanP.m_10, g_kalmanP.m_11,
        diff*LX16A_RADIAN_TO_DEGREE,
        g_currState.m_angle*LX16A_RADIAN_TO_DEGREE, g_currState.m_velocity*LX16A_RADIAN_TO_DEGREE);
}

static bool isServoInDeadZone(int16_t servoPos)
{
    return (servoPos < (int16_t)(LX16A_DEGREE_TO_SERVO_VALUE * -40.0f) ||
            servoPos > (int16_t)(LX16A_DEGREE_TO_SERVO_VALUE * 280.0f));
}

static void startStop()
{
    static float prevDegrees = 0.0f;
    static int prevTime = 0;
    static float prevVelocity = 0.0f;

    if (g_timer.read_ms() >= 5000)
    {
        g_servo.setRotationSpeed(0);
        g_pProcessRoutine = nothing;
        prevDegrees = 0.0f;
        prevTime = 0;
        prevVelocity = 0.0f;
    }

    int16_t position = g_servo.getPosition();
    int currTime = g_timer.read_us();

    float degrees = position * LX16A_SERVO_TO_DEGREE_VALUE;
    float deltaPosition = degrees - prevDegrees;
    uint32_t deltaTime = currTime - prevTime;
    float currVelocity = deltaPosition * 1000000.0f / deltaTime;
    float deltaVelocity = currVelocity - prevVelocity;
    float acceleration = deltaVelocity * 1000000.0f / deltaTime;

    printf("%d,%lu,%.2f,%.2f,%.2f,%.2f\n", currTime, deltaTime, degrees, deltaPosition, currVelocity, acceleration);

    prevTime = currTime;
    prevDegrees = degrees;
    prevVelocity = currVelocity;
}

static void nothing()
{
}


#if !MRI_ENABLE
static void serialReceiveISR()
{
    while (g_serial.readable())
    {
        g_cmd = g_serial.getc();
    }
}
#endif // !MRI_ENABLE
