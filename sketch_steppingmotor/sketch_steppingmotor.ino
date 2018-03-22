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

int32_t max_speed;
int32_t min_speed;
int32_t accel_rate_r;
int32_t target_speed = 0;
int32_t current_speed = 0;
int32_t cycle_wait;
long t1,t2;

char rcv_buf[256];
int rcv_len;
int motor_run = 0;

unsigned long run_step = 0;
unsigned long target_step = 0;

#if CMD_DEBUG==1
#define DebugPrint(...) Serial.print(__VA_ARGS__)
#define DebugPrintln(...) Serial.println(__VA_ARGS__)
#else
#define DebugPrint(...)
#define DebugPrintln(...)
#endif

#define SHIFT 8

void paramInit(){
  max_speed = (MAX_SPEED<<SHIFT);
  min_speed = (MIN_SPEED<<SHIFT);
  accel_rate_r = 2000000 / ACCEL_RATE;
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
      min_speed = ((long)param<<SHIFT);
      if(min_speed < (10<<SHIFT))
        min_speed = (10<<SHIFT);
      break;

    case 6: // setAccelRate rate(2bytes Hz / msec)
      DebugPrintln("cmd3: setAccelRate");
      if(param == 0)
        param = 1;
      accel_rate_r = 2000000 / param;
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
    cycle_wait = 1000000 / spd;
    int remain = target_step - run_step;
    int rlimit = spd * spd / accel_rate_r;
    long w = ((int64_t)accel_rate_r * cycle_wait * 2148) >> 24;       // long w = (int64_t)accel_rate_r * cycle_wait / (2000000/(1<<SHIFT));
    if(target_step != 0xffff && remain <= rlimit){
      current_speed -= w;
    }
    else {
      if(target_speed > current_speed){
        if((current_speed += w) > target_speed)
          current_speed = target_speed;
      }
      else if(target_speed < current_speed){
        if((current_speed -= w) < target_speed)
          current_speed = target_speed;
      }
    }

    if(current_speed < min_speed)
      current_speed = min_speed;
    digitalWrite(CLK, LOW); 
    t2 = micros();
    int d=cycle_wait - (t2-t1);
    if(d>0){
      delayMicroseconds(d);
      digitalWrite(LED, LOW);
    }
    else{
      digitalWrite(LED,HIGH);
    }
    run_step += 1;
    if (target_step != 0xffff && run_step >= target_step) {
      motor_run = 0;
      current_speed = 0;
    }
  }

}
