from lsm9ds0 import LSM9DS0

import signal
import sys


def signal_handler(signal, frame):
    device.shutdown()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

device = LSM9DS0(i2c_bus_num=1)
device.start()
signal.pause()
