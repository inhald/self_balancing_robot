#include "GYRO_DISCO_F429ZI.h"
#include "mbed.h"
#include "LCD_DISCO_F429ZI.h"

LCD_DISCO_F429ZI LCD;
GYRO_DISCO_F429ZI gyro;


//dc motors
PwmOut rightcw(PA_8);
PwmOut rightccw(PC_8);

PwmOut leftccw(PC_7);
PwmOut leftcw(PC_9);


bool stable = false;

int trial = 0;

//normalizing 
float mmax = -10000; float mmin = 10000;
float sum = 0; 


//reading gyro
uint8_t gyro_id;
Ticker gyroTicker;
float xyz[3];


int counter=0;


//average
float average[20];
float avg;
int index = 0;


//pid 
float feedback;
float kp = 40;



void clockwise(){
    rightccw.write(0.0f);
    leftccw.write(0.0f);

    rightcw.write(abs(kp*feedback));
    leftcw.write(abs(kp*feedback));
}

void counterclockwise(){

    rightcw.write(0.0f);
    leftcw.write(0.0f);


    leftccw.write(abs(kp*feedback));
    rightccw.write(abs(kp*feedback));
}

// void steady1(){
//     rightcw.write(kp*feedback);
//     leftcw.write(kp*feedback);
//     rightccw.write(0);
//     leftccw.write(0);
// }

// void steady2(){
//     rightcw=0;
//     leftcw=0;
//     rightccw=1;
//     leftccw=1;
// }


void steady(){
    rightccw.write(0.0f);
    rightcw.write(0.0f);
    leftcw.write(0.0f);
    leftccw.write(0.0f);
}


typedef enum {CLOCKWISE = 0, COUNTERCLOCKWISE = 1, STEADY = 2} State_Type;
State_Type current_state = STEADY;

void (*state_table[])(){clockwise, counterclockwise, steady};




void gyro_sample() { gyro.GetXYZ(xyz); }

int main() {
    __enable_irq();
  gyro.Init();
  gyro_id = gyro.ReadID();
  printf("%d\n", gyro_id);
  gyroTicker.attach(&gyro_sample, 100ms); 


  //dc motor init
  leftcw.period_us(256);
  leftccw.period_us(256);
  rightcw.period_us(256);
  rightccw.period_us(256);

  while (true) {
      LCD.Clear(LCD_COLOR_WHITE);

      float x = xyz[0];

      if(x>mmax){ mmax = x;}
      if(x<mmin){ mmin = x; }
      
      if(trial > 2){
        //angular acceleration
        float reading = (x-mmin)/(mmax-mmin);

        average[index%20] = reading;
        avg = 0;
        for(int i =0; i < 20; ++i) avg += average[i];
        avg=avg/20;
        ++index;



        uint8_t buffer[20];
        uint8_t buffer1[20];
        uint8_t buffer2[20];
        sprintf((char *)buffer, "%.3f", xyz[0]);
        sprintf((char *)buffer1, "%.3f", abs(feedback*kp));
        sprintf((char *)buffer2, "%.3f", avg);
        LCD.DisplayStringAt(0,80, (uint8_t *)&buffer, CENTER_MODE);
        LCD.DisplayStringAt(0,160, (uint8_t *)&buffer2, CENTER_MODE);
        LCD.DisplayStringAt(0,240, (uint8_t *)&buffer1, CENTER_MODE);

        //add tolerance

        feedback = reading-avg;

        if(feedback > 0.13){
            current_state = CLOCKWISE;
        }
        if (feedback < -0.13){
            current_state = COUNTERCLOCKWISE;
        }
        // else{
        //     current_state = STEADY;
        // }


      }

      state_table[current_state]();
      ThisThread::sleep_for(10ms);
      trial++;
  }
}

