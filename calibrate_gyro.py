import datetime
from lsm9ds0 import LSM9DS0

import signal
import numpy as np
import argparse
import math


class LSM9DS0GyroCalibrator(object):
    def __init__(self, gpio_int_pin_num, calibration_samples=1000, i2c_bus_num=0):

        self.device = LSM9DS0(self.data_callback, i2c_bus_num=i2c_bus_num, gyro_cal=[0., 0., 0.],
                              fifo_size=32, gpio_int_pin_num=gpio_int_pin_num)
        self.data = np.zeros((calibration_samples, 3))
        self.calibration_samples = calibration_samples
        self._calibration_sample_count = 0

    def start(self):
        self.device.start()

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
                print("Gyro Calibration Data: [%f, %f, %f]" % (np.average(d[:, 0]), np.average(d[:, 1]), np.average(d[:, 2])))
                print("All done!")
                self.device.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration-samples', help='Number of samples to collect for calibration', type=int,
                        default=1024)
    parser.add_argument('--i2c-bus', help='I2C bus number', type=int, default=0)
    parser.add_argument('--gpio-int-pin', type=int)
    args = parser.parse_args()

    if args.gpio_int_pin is None:
        raise ValueError('--gpio-int-pin must be set!')

    calibrator = LSM9DS0GyroCalibrator(gpio_int_pin_num=args.gpio_int_pin, calibration_samples=args.calibration_samples,
                                       i2c_bus_num=args.i2c_bus)
    calibrator.start()

