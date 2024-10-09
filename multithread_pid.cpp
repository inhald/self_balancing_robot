#include "GYRO_DISCO_F429ZI.h"
#include "mbed.h"
#include "LCD_DISCO_F429ZI.h"

// LCD and Gyroscope objects
LCD_DISCO_F429ZI LCD;
GYRO_DISCO_F429ZI gyro;

// DC motor PWM outputs
PwmOut rightcw(PA_8);
PwmOut rightccw(PC_8);
PwmOut leftccw(PC_7);
PwmOut leftcw(PC_9);

// Constants and global variables
const float kp = 40.0f;   // Proportional gain
const float ki = 10.0f;   // Integral gain
const float kd = 5.0f;    // Derivative gain
float xyz[3];
float avg = 0.0f;
float feedback = 0.0f;
uint8_t gyro_id;

// Mutexes for shared resources
Mutex xyz_mutex;
Mutex data_mutex;

// State definitions
typedef enum {CLOCKWISE = 0, COUNTERCLOCKWISE = 1, STEADY = 2} State_Type;
State_Type current_state = STEADY;

// Function pointers for state actions
typedef void (*StateFunction)(float output);
void clockwise(float output);
void counterclockwise(float output);
void steady(float output);
StateFunction state_table[] = {clockwise, counterclockwise, steady};

// Gyroscope reading thread
void gyro_thread() {
    while (true) {
        xyz_mutex.lock();
        gyro.GetXYZ(xyz);
        xyz_mutex.unlock();
        ThisThread::sleep_for(100ms);
    }
}

// Control thread for processing feedback and controlling motors
void control_thread() {
    float x;
    float mmax = -10000.0f, mmin = 10000.0f;
    float local_avg;
    float average[20] = {0};
    int index = 0;
    int trial = 0;
    float local_feedback;
    State_Type local_current_state = STEADY;

    // PID variables
    float error = 0.0f;
    float previous_error = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;
    float output = 0.0f;
    const float dt = 0.01f; // Time step in seconds (10ms)

    while (true) {
        xyz_mutex.lock();
        x = xyz[0];
        xyz_mutex.unlock();

        if (x > mmax) mmax = x;
        if (x < mmin) mmin = x;

        if (trial > 2) {
            float reading = (x - mmin) / (mmax - mmin);

            average[index % 20] = reading;
            local_avg = 0.0f;
            for (int i = 0; i < 20; ++i) local_avg += average[i];
            local_avg /= 20.0f;
            ++index;

            // Calculate PID components
            error = reading - local_avg;
            integral += error * dt;
            derivative = (error - previous_error) / dt;
            previous_error = error;

            // Anti-windup: Clamp integral term
            const float integral_limit = 1.0f; // Adjust as necessary
            if (integral > integral_limit) integral = integral_limit;
            if (integral < -integral_limit) integral = -integral_limit;

            // Compute PID output
            output = kp * error + ki * integral + kd * derivative;

            // Determine motor direction based on output
            if (output > 0.0f) {
                local_current_state = CLOCKWISE;
            } else if (output < 0.0f) {
                local_current_state = COUNTERCLOCKWISE;
            } else {
                local_current_state = STEADY;
            }

            data_mutex.lock();
            feedback = error;
            avg = local_avg;
            current_state = local_current_state;
            data_mutex.unlock();

            // Control motors
            state_table[local_current_state](output);
        }

        trial++;
        ThisThread::sleep_for(10ms);
    }
}

// LCD update thread
void lcd_thread() {
    float x, local_avg, local_feedback;

    while (true) {
        LCD.Clear(LCD_COLOR_WHITE);

        xyz_mutex.lock();
        x = xyz[0];
        xyz_mutex.unlock();

        data_mutex.lock();
        local_avg = avg;
        local_feedback = feedback;
        data_mutex.unlock();

        uint8_t buffer[20];
        uint8_t buffer1[20];
        uint8_t buffer2[20];
        sprintf((char *)buffer, "X: %.3f", x);
        sprintf((char *)buffer1, "Error: %.3f", local_feedback);
        sprintf((char *)buffer2, "Avg: %.3f", local_avg);

        LCD.DisplayStringAt(0, 80, buffer, CENTER_MODE);
        LCD.DisplayStringAt(0, 160, buffer2, CENTER_MODE);
        LCD.DisplayStringAt(0, 240, buffer1, CENTER_MODE);

        ThisThread::sleep_for(100ms);
    }
}

// State action functions
void clockwise(float output) {
    rightccw.write(0.0f);
    leftccw.write(0.0f);
    float pwm_value = fabs(output);
    if (pwm_value > 1.0f) pwm_value = 1.0f; // Clamp PWM value to max 1.0
    rightcw.write(pwm_value);
    leftcw.write(pwm_value);
}

void counterclockwise(float output) {
    rightcw.write(0.0f);
    leftcw.write(0.0f);
    float pwm_value = fabs(output);
    if (pwm_value > 1.0f) pwm_value = 1.0f; // Clamp PWM value to max 1.0
    leftccw.write(pwm_value);
    rightccw.write(pwm_value);
}

void steady(float output) {
    rightccw.write(0.0f);
    rightcw.write(0.0f);
    leftcw.write(0.0f);
    leftccw.write(0.0f);
}

int main() {
    __enable_irq();
    gyro.Init();
    gyro_id = gyro.ReadID();
    printf("Gyro ID: %d\n", gyro_id);

    // Initialize DC motors
    leftcw.period_us(256);
    leftccw.period_us(256);
    rightcw.period_us(256);
    rightccw.period_us(256);

    // Clear LCD
    LCD.Clear(LCD_COLOR_WHITE);

    // Start threads
    Thread gyroThread;
    Thread controlThread;
    Thread lcdThread;

    gyroThread.start(gyro_thread);
    controlThread.start(control_thread);
    lcdThread.start(lcd_thread);

    // Main thread idle loop
    while (true) {
        ThisThread::sleep_for(1000ms);
    }
}
