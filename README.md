# lsm9ds0
IRQ/FIFO based LSM9DS0 ROS Node

Requires connecting the DRDY (data ready) signal of the LSM9DS0 to a GPIO, and the ROS node must have permission to access the pin in /sys/class/gpio. It is a good idea to pre-export the pin in /etc/rc.local:

```
echo 17 > /sys/class/gpio/export
chmod 777 /sys/class/gpio/gpio17/*
```

## Nodes

### lsm9ds0_node.py

#### Parameters

| Param | Type  | Description  |
| :------------- |:-------------| :-----|
| i2c_bus_num | int | i2c device bus number. For /dev/i2c-1, set to 1 |
| calibrate | bool | Whether to calibrate the gyroscope on startup |
| gpio_int_pin_num | int | /sys/class/gpio GPIO number to receive the data ready interupt on |
| gyro_cal_x | float | gyroscope x axis bias |
| gyro_cal_y | float | gyroscope y axis bias |
| gyro_cal_z | float | gyroscope z axis bias |

#### Topics

| Action | Topic | Type |
| :------------- |:-------------| :-----|
| publish | /imu/data_raw | sensor_msgs/Imu |
| publish | /imu/mag | sensor_msgs/MagneticField |
