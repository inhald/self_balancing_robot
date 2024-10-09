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
const float kp = 40.0f;
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
typedef void (*StateFunction)(float feedback);
void clockwise(float feedback);
void counterclockwise(float feedback);
void steady(float feedback);
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

            local_feedback = reading - local_avg;

            if (local_feedback > 0.13f) {
                local_current_state = CLOCKWISE;
            } else if (local_feedback < -0.13f) {
                local_current_state = COUNTERCLOCKWISE;
            } else {
                local_current_state = STEADY;
            }

            data_mutex.lock();
            feedback = local_feedback;
            avg = local_avg;
            current_state = local_current_state;
            data_mutex.unlock();

            // Control motors
            state_table[local_current_state](local_feedback);
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
        sprintf((char *)buffer, "%.3f", x);
        sprintf((char *)buffer1, "%.3f", abs(local_feedback * kp));
        sprintf((char *)buffer2, "%.3f", local_avg);

        LCD.DisplayStringAt(0, 80, buffer, CENTER_MODE);
        LCD.DisplayStringAt(0, 160, buffer2, CENTER_MODE);
        LCD.DisplayStringAt(0, 240, buffer1, CENTER_MODE);

        ThisThread::sleep_for(100ms);
    }
}

// State action functions
void clockwise(float feedback) {
    rightccw.write(0.0f);
    leftccw.write(0.0f);
    float pwm_value = abs(kp * feedback);
    rightcw.write(pwm_value);
    leftcw.write(pwm_value);
}

void counterclockwise(float feedback) {
    rightcw.write(0.0f);
    leftcw.write(0.0f);
    float pwm_value = abs(kp * feedback);
    leftccw.write(pwm_value);
    rightccw.write(pwm_value);
}

void steady(float feedback) {
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
