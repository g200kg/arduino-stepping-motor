// Stepping Motor Driver with I2C Slave Interface for Arduino
//
// This software runs arudino as an i2c slave device.
// It works by sending commands from i2c host device connected to arduino
//   * Supposed to use with "EasyDriver" or some other DIR/CLK type driver chip.
//   * Single axis stepping motor driver.
//   * Trapezoidal acceleration / deceleration are available.
//   * Default I2C address is 0x12. defined by I2CADDR.
//   * Arduino to DriverChip connection :
//     D 8  => CLK (microstep clock)
//     D 9  => DIR (motor direction)
//     D10  => RST (enable stepper LOW:disable HIGH:enable)
// 
// I2C commands
// all i2c write commands have 2 bytes parameter (little endian)
//   Write:
//   |   commands         | param1   | param2   |  description                                            |
//   |--------------------|----------|----------|---------------------------------------------------------|
//   | 0x00 | ParamInit   | dummy    | dummy    | Set motor profile to default value                      |
//   | 0x01 | Forward     | steps(L) | steps(H) | Move forward a specified number of steps                |
//   |      |             |          |          |  if number is 0xffff move forward infinity steps        |
//   | 0x02 | Reverse     | steps(L) | steps(H) | Move backward a specified number of steps               |
//   |      |             |          |          |  if number is 0xffff move backward infinity steps       |
//   | 0x03 | Abort       | dummy    | dummy    | Cancel the currently running move                       |
//   | 0x04 | SetSpeed    | speed(L) | speed(H) | Set motor speed (micro steps per sec) default is 3200   |
//   | 0x05 | SetMinSpeed | speed(L) | speed(H) | Set min speed (micro steps per sec) default is 800      |
//   | 0x06 | SetAccelRate| rate(L)  | rate(H)  | Accel/DeAccel rate. msec / kHz.                         |
//   |      |             |          |          |  if 0, accel/deaccel is disabled. default is 20.        |
//   | 0x07 | Enable      | 0/1      | dummy    | enable / disable stepper.                               |
//
//   Read:
//     Any read access return 1 byte motor status 0x00:stopped 0x01:moving
//
//----
// Test code for Raspberry Pi Python
//
// import smbus
// i2c = smbus.SMBus(1)
// # move forward 3200 (=0xc80) steps
// i2c.write_i2c_block_data(0x12, 1, [0x80, 0x0c])
// 

#include <Wire.h>

// for Debug
#define CMD_DEBUG 0

// I2C address
#define I2CADDR 0x12

// Arduino pins
#define CLK 8
#define DIR 9
#define ENA 10
//
#define LED 13

// stepping motor profiles (8 micro step)
#define MAX_SPEED  (3200)           // default motor speed. microsteps per sec
#define MIN_SPEED  (800)            // default min speed. microsteps per sec
#define ACCEL_RATE (20)             // default accel rate. inc/dec ratio of msec/kHz

long max_speed;
long min_speed;
long min_spd;
long min_spd2;
long accel_rate;
long accel_rate_r;
long accel_rate_b;
long target_speed = 0;
long current_speed = 0;
long cycle_wait;
long rlimit;
long delta_speed;
unsigned long t1,t2;

char rcv_buf[256];
int rcv_len;
int motor_run = 0;

unsigned int run_step = 0;
unsigned int target_step = 0;

#if CMD_DEBUG==1
#define DebugPrint(...) Serial.print(__VA_ARGS__)
#define DebugPrintln(...) Serial.println(__VA_ARGS__)
#else
#define DebugPrint(...)
#define DebugPrintln(...)
#endif

#define SHIFT 7

void paramInit(){
  max_speed = ((unsigned long)MAX_SPEED<<SHIFT);
  min_spd = MIN_SPEED;
  min_spd2 = (unsigned long)MIN_SPEED * MIN_SPEED;
  min_speed = ((unsigned long)MIN_SPEED<<SHIFT);
  accel_rate = ACCEL_RATE;
  accel_rate_r = (unsigned long)1000000 / ACCEL_RATE;
  accel_rate_b = accel_rate * 16384 / 15625;
}

void setup() {
  paramInit();
  pinMode(CLK, OUTPUT); 
  pinMode(DIR, OUTPUT); 
  pinMode(ENA, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(CLK, LOW); 
  digitalWrite(DIR, LOW); 
  digitalWrite(ENA, HIGH);
  digitalWrite(LED, LOW);
  Wire.begin(I2CADDR);            // join i2c bus
  Wire.onReceive(receiveEvent);   // I2C write
  Wire.onRequest(requestEvent);   // I2C read
  
#if CMD_DEBUG==1
  Serial.begin(115200);          // start serial for output
  Serial.println("start");       // print the character
#endif

}

void requestEvent() { // I2C read

  DebugPrint("request ");
  DebugPrint(motor_run);         // print the integer
  DebugPrintln("");

  Wire.write(motor_run);
}

void receiveEvent(int howMany) {
  int i = 0;
  int p;
  int busy = rcv_len;
  while (0 < Wire.available()) { // loop through all but the last
    if (busy == 0) {
      rcv_buf[i++] = Wire.read(); // receive byte as a character
    } else {
      int a = Wire.read(); // dummy read
    }
  }
  if (i == 3) {
    rcv_len = i;
  }

  DebugPrint("receive ");
  DebugPrint(i);         // print the integer
  DebugPrintln(" bytes");

}
void loop() {
  if(motor_run)
    digitalWrite(CLK, HIGH); 
  t1 = micros();
  if( rcv_len > 0 ) {
    DebugPrint("data[");
    DebugPrint(rcv_buf[0] & 0xff,HEX);
    DebugPrint(",");
    DebugPrint(rcv_buf[1] & 0xff,HEX);
    DebugPrint(",");
    DebugPrint(rcv_buf[2] & 0xff,HEX);
    DebugPrintln("]");   

    unsigned int param = ((rcv_buf[2]&0xff)<<8)+(rcv_buf[1]&0xff);  // little endian 2bytes param
    switch(rcv_buf[0]) {

    case 0: // param init
      DebugPrintln("cmd0: paramInit");
      paramInit();
      break;

    case 1: // forward, steps(2bytes)
      DebugPrintln("cmd1: forward");
      if(motor_run == 0) {
        digitalWrite(DIR, LOW); 
        run_step = 0;
        target_step = param;
        motor_run = 1;
        current_speed = min_speed;
        target_speed = max_speed;
      } else {
        DebugPrintln("busy");
      }
      break;

    case 2: // reverse, steps(2bytes)
      DebugPrintln("cmd1: reverse");
      if(motor_run == 0) {
        digitalWrite(DIR, HIGH); 
        run_step = 0;
        target_step = param;
        motor_run = 1;
        current_speed = min_speed;
        target_speed = max_speed;
      } else {
        DebugPrintln("busy");
      }
      break;
      
    case 3: // abort, dummy(2bytes)
      DebugPrintln("cmd3: abort");
      motor_run = 0;
      current_speed = 0;
      target_speed = 0;
      break;
      
    case 4: // setSpeed, speed(2bytes pulse per sec)
      DebugPrintln("cmd4: setSpeed");
      max_speed = ((long)param<<SHIFT);
      target_speed = max_speed;
      break;

    case 5: // setMinSpeed speed(2bytes pulse per sec)
      DebugPrintln("cmd3: setMinSpeed");
      if(param < 10)
        param = 10;
      min_spd = (long)param;
      min_spd2 = min_spd * min_spd;
      min_speed = (min_spd<<SHIFT);
      break;

    case 6: // setAccelRate rate(2bytes msec / kHz)
      DebugPrintln("cmd3: setAccelRate");
      if(param == 0)
        param = 1;
      accel_rate = param;
      accel_rate_r = 1000000 / param;
      accel_rate_b = accel_rate * 16384 / 15625;
      break;

    case 7: // enable
      DebugPrintln("cmd3: enable");
      if(param&1)
        digitalWrite(ENA, HIGH);
      else
        digitalWrite(ENA, LOW);
      break;
    }
    rcv_len = 0;
  }
  
  if(motor_run){
    long spd = current_speed>>SHIFT;
    int remain = target_step - run_step;
    rlimit = (((spd * spd - min_spd2)>>7) * accel_rate_b) >> 14;
    cycle_wait = (unsigned long)1000000 / spd;
    delta_speed = (unsigned long)128000000 / (spd * accel_rate);
    if(target_step == 0xffff)
      remain = 0x7fff;
    if(remain <= rlimit){
      if(remain <= 0){
        motor_run = 0;
        current_speed = 0;
      }
      else
        current_speed -= delta_speed;
    }
    else {
      if(target_speed > current_speed){
        if((current_speed += delta_speed) > target_speed)
          current_speed = target_speed;
      }
      else if(target_speed < current_speed){
        if((current_speed -= delta_speed) < target_speed)
          current_speed = target_speed;
      }
    }

    if(current_speed < min_speed)
      current_speed = min_speed;
    digitalWrite(CLK, LOW); 
    t2 = micros();
    int d=(long)cycle_wait - (long)(t2-t1);
    if(d>0){
      delayMicroseconds(d);
      digitalWrite(LED, LOW);
    }
    else{
      digitalWrite(LED,HIGH);
    }
    ++run_step;
  }
}
