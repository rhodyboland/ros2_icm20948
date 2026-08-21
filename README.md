# ros2_icm20948
Driver for the ICM-20948 IMU

## Dependencies
```bash
pip3 install sparkfun-qwiic-icm20948
```

## Permissions
In order to run this node, i2c access permissions must be granted to the user that runs it. To do so run the following command: 
```bash
sudo adduser <your_user> i2c
```

## Parameters
`i2c_bus` selects the Linux I2C bus passed to the SparkFun Qwiic driver. For example, `i2c_bus: 7` uses `/dev/i2c-7`.

On Jetson, prefer `i2cdetect -y -r 7` when checking the bus. Plain `i2cdetect -y 7` uses SMBus quick writes, which may be unsupported and can miss valid devices.
