import importlib
import math

import qwiic_icm20948
import qwiic_i2c
import rclpy
import sensor_msgs.msg
from rclpy.node import Node


class ICM20948Node(Node):
    def __init__(self):
        super().__init__("icm20948_node")

        # Logger
        self.logger = self.get_logger()

        # Parameters
        self.declare_parameter("i2c_bus", 1)
        i2c_bus = self.get_parameter("i2c_bus").get_parameter_value().integer_value
        self.i2c_bus = i2c_bus

        self.declare_parameter("i2c_address", 0x69)
        i2c_addr = self.get_parameter("i2c_address").get_parameter_value().integer_value
        self.i2c_addr = i2c_addr

        self.declare_parameter("frame_id", "imu_icm20948")
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_id = frame_id

        self.declare_parameter("pub_rate", 50)
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.pub_rate = pub_rate

        # IMU instance
        i2c_driver = self._get_i2c_driver(self.i2c_bus)
        self.imu = qwiic_icm20948.QwiicIcm20948(
            address=self.i2c_addr,
            i2c_driver=i2c_driver,
        )
        if not self.imu.begin():
            raise RuntimeError(
                "Failed to initialize ICM20948 on "
                f"I2C bus {self.i2c_bus} at address 0x{self.i2c_addr:02X}."
            )
        self.imu.setFullScaleRangeGyro(qwiic_icm20948.dps2000)
        self.imu.setFullScaleRangeAccel(qwiic_icm20948.gpm16)

        # Publishers
        self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(
            sensor_msgs.msg.MagneticField, "/imu/mag_raw", 10
        )
        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def _get_i2c_driver(self, i2c_bus):
        if hasattr(qwiic_i2c, "get_i2c_driver"):
            i2c_driver = qwiic_i2c.get_i2c_driver(iBus=i2c_bus)
            return self._require_i2c_driver(i2c_driver, i2c_bus)

        if hasattr(qwiic_i2c, "getI2CDriver"):
            try:
                i2c_driver = qwiic_i2c.getI2CDriver(iBus=i2c_bus)
                return self._require_i2c_driver(i2c_driver, i2c_bus)
            except TypeError as exc:
                i2c_driver = self._get_legacy_linux_i2c_driver(i2c_bus)
                if i2c_driver is not None:
                    return i2c_driver

                raise RuntimeError(
                    "Installed qwiic_i2c does not expose I2C bus selection. "
                    "Upgrade sparkfun-qwiic-i2c, or patch qwiic_i2c to use bus "
                    f"{i2c_bus}."
                ) from exc

        raise RuntimeError("Unable to find a SparkFun qwiic_i2c driver factory.")

    def _get_legacy_linux_i2c_driver(self, i2c_bus):
        if not hasattr(qwiic_i2c, "LinuxI2C"):
            return None

        if hasattr(qwiic_i2c, "_theDriver"):
            qwiic_i2c._theDriver = None

        linux_i2c = importlib.import_module(qwiic_i2c.LinuxI2C.__module__)

        # Older qwiic_i2c releases hard-code bus 1 inside this connection helper.
        def connect_to_selected_bus():
            try:
                import smbus2
            except Exception as exc:
                raise RuntimeError(
                    "Unable to load smbus2 for SparkFun I2C."
                ) from exc

            try:
                return smbus2.SMBus(i2c_bus)
            except Exception as exc:
                raise RuntimeError(
                    f"Failed to connect to I2C bus {i2c_bus}."
                ) from exc

        linux_i2c._connectToI2CBus = connect_to_selected_bus
        return self._require_i2c_driver(qwiic_i2c.LinuxI2C(), i2c_bus)

    def _require_i2c_driver(self, i2c_driver, i2c_bus):
        if i2c_driver is None:
            raise RuntimeError(
                f"Unable to create SparkFun I2C driver for bus {i2c_bus}."
            )
        return i2c_driver

    def publish_cback(self):
        imu_msg = sensor_msgs.msg.Imu()
        mag_msg = sensor_msgs.msg.MagneticField()
        if self.imu.dataReady():
            try:
                self.imu.getAgmt()
            except Exception as e:
                self.logger.info(str(e))

            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            imu_msg.linear_acceleration.x = self.imu.axRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.y = self.imu.ayRaw * 9.81 / 2048.0
            imu_msg.linear_acceleration.z = self.imu.azRaw * 9.81 / 2048.0
            imu_msg.angular_velocity.x = self.imu.gxRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.y = self.imu.gyRaw * math.pi / (16.4 * 180)
            imu_msg.angular_velocity.z = self.imu.gzRaw * math.pi / (16.4 * 180)
            imu_msg.orientation_covariance[0] = -1

            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = self.imu.mxRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.y = self.imu.myRaw * 1e-6 / 0.15
            mag_msg.magnetic_field.z = self.imu.mzRaw * 1e-6 / 0.15

        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    icm20948_node = ICM20948Node()
    rclpy.spin(icm20948_node)

    icm20948_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
