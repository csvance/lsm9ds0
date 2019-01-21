#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField
from threading import Event
import numpy as np
import tf
import sys

from lsm9ds0 import LSM9DS0


class LSM9DS0Node(object):
    def __init__(self):
        rospy.init_node('lsm9ds0')

        i2c_bus_num = rospy.get_param('~i2c_bus_num')
        gpio_int_pin_num = rospy.get_param('~gpio_int_pin_num')

        if rospy.get_param('~compute_covariance', False):
            calculator = LSM9DS0CovarianceCalculator(gpio_int_pin_num=gpio_int_pin_num,
                                               i2c_bus_num=i2c_bus_num)
            calculator.start()
            calculator.wait()
            sys.exit(0)
        else:
            self.linear_acceleration_covariance = [float(f) for f in rospy.get_param('~linear_acceleration_covariance', '0, 0, 0, 0, 0, 0, 0, 0, 0').split(',')]
            self.angular_velocity_covariance = [float(f) for f in rospy.get_param('~angular_velocity_covariance', '0, 0, 0, 0, 0, 0, 0, 0, 0').split(',')]
            self.magnetic_field_covariance = [float(f) for f in rospy.get_param('~magnetic_field_covariance', '0, 0, 0, 0, 0, 0, 0, 0, 0').split(',')]

        self._tf_broadcaster = tf.TransformBroadcaster()

        if rospy.get_param('~calibrate_gyro'):
            calibrator = LSM9DS0GyroCalibrator(gpio_int_pin_num=gpio_int_pin_num,
                                               i2c_bus_num=i2c_bus_num)
            calibrator.start()
            calibrator.wait()
            sys.exit(0)
        else:
            g_cal_x = rospy.get_param('~gyro_cal_x')
            g_cal_y = rospy.get_param('~gyro_cal_y')
            g_cal_z = rospy.get_param('~gyro_cal_z')

        self._publisher_imu = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        self._publisher_magnetic = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

        self._sensor = LSM9DS0(callback=self._sensor_callback, gyro_cal=[g_cal_x, g_cal_y, g_cal_z],
                               i2c_bus_num=i2c_bus_num, gpio_int_pin_num=gpio_int_pin_num, fifo_size=1)
        self._sensor.start()

    def _sensor_callback(self, accelerometer, magnometer, gyrometer):

        if rospy.is_shutdown():
            return

        end_ts = rospy.get_time()

        for i in range(0, len(accelerometer)):

            imu = Imu()
            magnetic = MagneticField()

            # Calculate timestamp for reading
            samples_behind = (len(accelerometer) - 1) - i
            samples_per_sec = len(accelerometer) / 50.
            stamp = rospy.Time.from_sec(end_ts - samples_behind * samples_per_sec)

            imu.header.stamp = stamp
            imu.header.frame_id = "imu"
            magnetic.header.stamp = stamp

            imu.linear_acceleration.x = accelerometer[i][0]
            imu.linear_acceleration.y = accelerometer[i][1]
            imu.linear_acceleration.z = accelerometer[i][2]
            imu.linear_acceleration_covariance = self.linear_acceleration_covariance

            imu.angular_velocity.x = gyrometer[i][0]
            imu.angular_velocity.y = gyrometer[i][1]
            imu.angular_velocity.z = gyrometer[i][2]
            imu.angular_velocity_covariance = self.angular_velocity_covariance

            magnetic.magnetic_field.x = magnometer[i][0]
            magnetic.magnetic_field.y = magnometer[i][1]
            magnetic.magnetic_field.z = magnometer[i][2]
            magnetic.magnetic_field_covariance = self.magnetic_field_covariance

            try:
                self._tf_broadcaster.sendTransform((0, 0, 0),
                                       (0., 0., 0., 1.),
                                       imu.header.stamp,
                                       "base_link", "imu")
                self._publisher_imu.publish(imu)
                self._publisher_magnetic.publish(magnetic)
            except rospy.ROSException:
                pass

    def shutdown(self):
        self._sensor.shutdown()


class LSM9DS0GyroCalibrator(object):
    def __init__(self, gpio_int_pin_num, calibration_samples=1024*10, i2c_bus_num=0):

        self.device = LSM9DS0(self.data_callback, i2c_bus_num=i2c_bus_num, gyro_cal=[0., 0., 0.],
                              fifo_size=32, gpio_int_pin_num=gpio_int_pin_num)
        self.done = Event()
        self.data = np.zeros((calibration_samples, 3))
        self.calibration_samples = calibration_samples
        self._calibration_sample_count = 0

    def start(self):
        self.device.start()

    def wait(self):
        self.done.wait()

    def data_callback(self, accel_data, mag_data, gyro_data):
        d = self.data

        for i in range(0, len(gyro_data)):
            # Log gyro data
            x, y, z = gyro_data[i]
            d[self._calibration_sample_count][0] = x
            d[self._calibration_sample_count][1] = y
            d[self._calibration_sample_count][2] = z

            self._calibration_sample_count += 1

            if self._calibration_sample_count >= self.calibration_samples:
                for idx, col in enumerate(['x', 'y', 'z']):
                    print("<param name=\"~gyro_cal_%s\" value=\"%.8f\"/>" % (col, np.average(d[:, idx])))
                print("All done!")

                self.calibration = (np.average(d[:, 0]), np.average(d[:, 1]), np.average(d[:, 2]))
                self.device.shutdown()
                self.done.set()


class LSM9DS0CovarianceCalculator(object):
    def __init__(self, gpio_int_pin_num, calibration_samples=1024*10, i2c_bus_num=0):

        self.device = LSM9DS0(self.data_callback, i2c_bus_num=i2c_bus_num, gyro_cal=[0., 0., 0.],
                              fifo_size=32, gpio_int_pin_num=gpio_int_pin_num)
        self.done = Event()

        self.linear_readings = np.zeros((calibration_samples, 3))
        self.angular_readings = np.zeros((calibration_samples, 3))
        self.magnetometer_readings = np.zeros((calibration_samples, 3))
        self.calibration_samples = calibration_samples
        self.samples_index = 0

    def start(self):
        self.device.start()

    def wait(self):
        self.done.wait()

    def data_callback(self, accelerometer, magnometer, gyrometer):

        for i in range(0, len(gyrometer)):

            self.linear_readings[self.samples_index] = accelerometer[i][0], accelerometer[i][1], accelerometer[i][2]
            self.angular_readings[self.samples_index] = gyrometer[i][0], gyrometer[i][1], gyrometer[i][2]
            self.magnetometer_readings[self.samples_index] = magnometer[i][0], magnometer[i][1], magnometer[i][2]

            self.samples_index += 1
            if self.samples_index == self.calibration_samples:

                self.linear_acceleration_covariance = np.cov(np.transpose(self.linear_readings)).reshape(9).tolist()
                self.angular_velocity_covariance = np.cov(np.transpose(self.angular_readings)).reshape(9).tolist()
                self.magnetic_field_covariance = np.cov(np.transpose(self.magnetometer_readings)).reshape(9).tolist()

                print("<param name=\"~linear_acceleration_covariance\" value=\"%s\"/>" % (", ".join(["%.8f" % f for f in self.linear_acceleration_covariance])))
                print("<param name=\"~angular_velocity_covariance\" value=\"%s\"/>" % (", ".join(["%.8f" % f for f in self.angular_velocity_covariance])))
                print("<param name=\"~magnetic_field_covariance\" value=\"%s\"/>" % (", ".join(["%.8f" % f for f in self.magnetic_field_covariance])))

                self.device.shutdown()
                self.done.set()

                print("All done!")


if __name__ == "__main__":
    node = LSM9DS0Node()
    rospy.spin()
    node.shutdown()
