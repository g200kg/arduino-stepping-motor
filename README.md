# arduino-stepping-motor
Stepping motor driver with I2C Slave I/F for Arduino


This software runs arudino as an i2c slave device.  
It works by sending commands from i2c host device connected to arduino  

* Supposed to use with "EasyDriver" or some other DIR/CLK type driver module/chip.  
e.g.  [sparkfun : EasyDriver](https://www.sparkfun.com/products/12779)
* Single axis stepping motor driver.
* Trapezoidal acceleration / deceleration are available.
* Default I2C address is 0x12.
* Arduino to DriverChip connection :
  * D 8  => CLK (microstep clock)
  * D 9  => DIR (motor direction)
  * D10  => RST (enable stepper LOW:disable HIGH:enable)

## I2C commands
all i2c write commands have 2 bytes parameter (little endian)  
### Write:
|   commands         || param1   | param2   |  description                                            |
|--------------------|----------|----------|---------------------------------------------------------|
| 0x00 | ParamInit   | dummy    | dummy    | Set motor profile to default value                      |
| 0x01 | Forward     | steps(L) | steps(H) | Move forward a specified number of steps                |
|      |             |          |          |  if number is 0xffff move forward infinity steps        |
| 0x02 | Reverse     | steps(L) | steps(H) | Move backward a specified number of steps               |
|      |             |          |          |  if number is 0xffff move backward infinity steps       |
| 0x03 | Abort       | dummy    | dummy    | Cancel the currently running move                       |
| 0x04 | SetSpeed    | speed(L) | speed(H) | Set motor speed (micro steps per sec) default is 3200   |
| 0x05 | SetMinSpeed | speed(L) | speed(H) | Set min speed (micro steps per sec) default is 800      |
| 0x06 | SetAccelRate| rate(L)  | rate(H)  | Accel/DeAccel rate. Hz / msec. default is 20            |
| 0x07 | Enable      | 0/1      | dummy    | enable / disable stepper                                |


### Read:
  Any read access return 1 byte motor status 0x00:stopped 0x01:moving

----
## Test code for Raspberry Pi Python

```python
 import smbus
 i2c = smbus.SMBus(1)
 # move forward 3200 (=0xc80) steps
 i2c.write_i2c_block_data(0x12, 1, [0x80, 0x0c])
```

## Demo Movie
[Demo Movie](./steppertest.mp4)

## License
This software is based on
a project of [Kazutaka Nakamura](http://makaizou.blogspot.jp/)'s stepping motor driver.

Released under the [MIT](./LICENSE) license
