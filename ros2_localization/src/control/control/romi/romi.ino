#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>

float DT = 0.001;
float ENCODER = 12*3952/33;
float Kp = 3;
float Ki = 0.9;
float Kd = 8;

struct Data
{

  float wl_desired; //20
  float wr_desired; //24 bytes


  float wl_measured; //28
  float wr_measured; //32
  uint16_t batteryMillivolts; //36


  
};

PololuRPiSlave<struct Data,5> slave;
PololuBuzzer buzzer;
Romi32U4Motors motors;
Romi32U4Encoders encoders;



void setup() {
  // put your setup code here, to run once:
  // Set up the slave at I2C address 20.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");

}

float I_left = 0;
float I_right = 0;

float e_prev_r = 0;
float e_prev_l = 0;


void loop() {
  // put your main code here, to run repeatedly:
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();


  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivolts();

  float wl_measured = get_w_left();
  float wr_measured = get_w_right();

  slave.buffer.wl_measured = wl_measured;
  slave.buffer.wr_measured = wr_measured;

  motors.setSpeeds(pid_control(slave.buffer.wl_desired, wl_measured, &I_left, &e_prev_l), pid_control(slave.buffer.wl_desired, wr_measured, &I_right, &e_prev_r));

  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();

  delay(DT);

}

inline float get_w_left() {

  float w_left = 2*PI*encoders.getCountsAndResetLeft()/DT;

  return w_left;


}

inline float get_w_right() {

  float w_right = 2*PI*encoders.getCountsAndResetRight()/DT;

  return w_right;


}

inline float pid_control(float w_desired, float w_measured, float *I, float *e_prev) {


  float e = w_desired-w_measured;

  float duty_cycle = min(max(Kp*e + (*I + Ki*e) + Kd*(e - *e_prev), -200), 200);

  *e_prev = e;

  return duty_cycle;

}



