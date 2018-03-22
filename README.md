# arduino-stepping-motor
Stepping motor driver with I2C Slave I/F for Arduino


This software runs arudino as an i2c slave device.  
It works by sending commands from i2c host device connected to arduino  

* Supposed to use with "EasyDriver" or some other DIR/CLK type driver module/chip.  
e.g.  [sparkfun : EasyDriver](https://www.sparkfun.com/products/12779)
* Single axis stepping motor driver.
* Trapezoidal acceleration / deceleration are available.
* Default I2C address is 0x12.
* Pulse speed up to about 7000 pulses / sec.
* Arduino to DriverChip connection :
  * D 8  => CLK (microstep clock)
  * D 9  => DIR (motor direction)
  * D10  => RST (enable stepper LOW:disable HIGH:enable)

## I2C commands
all i2c write commands have 2 bytes parameter (little endian)  
### Write:
|      |commands     | param1   | param2   |  description                                            |
|------|-------------|----------|----------|---------------------------------------------------------|
| 0x00 | ParamInit   | dummy    | dummy    | Set motor profile to default value                      |
| 0x01 | Forward     | steps(L) | steps(H) | Move forward a specified number of steps<br/>if number is 0xffff move forward infinity steps        |
| 0x02 | Reverse     | steps(L) | steps(H) | Move backward a specified number of steps<br/>if number is 0xffff move backward infinity steps       |
| 0x03 | Abort       | dummy    | dummy    | Cancel the currently running move                       |
| 0x04 | SetSpeed    | speed(L) | speed(H) | Set motor speed (micro steps per sec) default is 3200   |
| 0x05 | SetMinSpeed | speed(L) | speed(H) | Set min speed (micro steps per sec) default is 800      |
| 0x06 | SetAccelRate| rate(L)  | rate(H)  | Accel/DeAccel rate. msec / kHz. if 0, accel/deaccel is disabled. default is 20            |
| 0x07 | Enable      | 0/1      | dummy    | enable / disable stepper                                |


### Read:
  Any read access return 1 byte motor status 0x00:stopped 0x01:moving

----
## Motor Speed Graph

![](./steppingmotor.png)

----
## Test with I2CTools

```
i2cset -y 1 0x12 0x01 1600 w
```

## Demo Movie
[![Demo Movie](https://img.youtube.com/vi/KaYRTRlLtlk/0.jpg)](https://www.youtube.com/watch?v=KaYRTRlLtlk)

## License
This software is based on
a project of [Kazutaka Nakamura](http://makaizou.blogspot.jp/)'s stepping motor driver.

Released under the MIT license
