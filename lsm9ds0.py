from threading import Thread, Event
from Queue import Queue
from sysfs_gpio import GPIO
from smbus import SMBus


class LSM9DS0(object):
    XM_ADDRESS = 0x1D
    G_ADDRESS = 0x6B

    WHO_AM_I_G = 0x0F
    CTRL_REG1_G = 0x20
    CTRL_REG2_G = 021
    CTRL_REG3_G = 0x22
    CTRL_REG4_G = 0x23
    CTRL_REG5_G = 0x24
    OUT_X_L_G = 0x28
    OUT_X_H_G = 0x29
    OUT_Y_L_G = 0x2A
    OUT_Y_H_G = 0x2B
    OUT_Z_L_G = 0x2C
    OUT_Z_H_G = 0x2D

    # 2D gyroscope low-high register tuple
    OUT_XYZ_LH_G = ((OUT_X_L_G, OUT_X_H_G),
                    (OUT_Y_L_G, OUT_Y_H_G),
                    (OUT_Z_L_G, OUT_Z_H_G))

    # LSM9DS0 temperature addresses
    OUT_TEMP_L_XM = 0x05
    OUT_TEMP_H_XM = 0x06

    # Temperature low-high register tuple
    OUT_TEMP_LH = (OUT_TEMP_L_XM, OUT_TEMP_H_XM)

    # Magnetometer addresses
    STATUS_REG_M = 0x07
    OUT_X_L_M = 0x08
    OUT_X_H_M = 0x09
    OUT_Y_L_M = 0x0A
    OUT_Y_H_M = 0x0B
    OUT_Z_L_M = 0x0C
    OUT_Z_H_M = 0x0D

    # 2D magnetometer low-high register tuple
    OUT_XYZ_LH_M = ((OUT_X_L_M, OUT_X_H_M),
                    (OUT_Y_L_M, OUT_Y_H_M),
                    (OUT_Z_L_M, OUT_Z_H_M))

    # Shared (mag and accel) addresses
    WHO_AM_I_XM = 0x0F
    INT_CTRL_REG_M = 0x12
    INT_SRC_REG_M = 0x13
    CTRL_REG0_XM = 0x1F
    CTRL_REG1_XM = 0x20
    CTRL_REG2_XM = 0x21
    CTRL_REG3_XM = 0x22
    CTRL_REG4_XM = 0x23
    CTRL_REG5_XM = 0x24
    CTRL_REG6_XM = 0x25
    CTRL_REG7_XM = 0x26

    FIFO_CTRL_REG = 0x2E
    FIFO_CTRL_REG_G = 0x2E

    # Accelerometer addresses
    OUT_X_L_A = 0x28
    OUT_X_H_A = 0x29
    OUT_Y_L_A = 0x2A
    OUT_Y_H_A = 0x2B
    OUT_Z_L_A = 0x2C
    OUT_Z_H_A = 0x2D

    # 2D accelerometer low-high register tuple
    OUT_XYZ_LH_A = ((OUT_X_L_A, OUT_X_H_A),
                    (OUT_Y_L_A, OUT_Y_H_A),
                    (OUT_Z_L_A, OUT_Z_H_A))

    # Various settings included in the Arduino library. I haven't used these,
    # to keep to a default setting for simplicity, however, users can change
    # the settings easily.
    ACCELRANGE_2G = 0b000 << 3
    ACCELRANGE_4G = 0b001 << 3
    ACCELRANGE_6G = 0b010 << 3
    ACCELRANGE_8G = 0b011 << 3
    ACCELRANGE_16G = 0b100 << 3

    ACCELDATARATE_POWERDOWN = 0b0000 << 4
    ACCELDATARATE_3_125HZ = 0b0001 << 4
    ACCELDATARATE_6_25HZ = 0b0010 << 4
    ACCELDATARATE_12_5HZ = 0b0011 << 4
    ACCELDATARATE_25HZ = 0b0100 << 4
    ACCELDATARATE_50HZ = 0b0101 << 4
    ACCELDATARATE_100HZ = 0b0110 << 4
    ACCELDATARATE_200HZ = 0b0111 << 4
    ACCELDATARATE_400HZ = 0b1000 << 4
    ACCELDATARATE_800HZ = 0b1001 << 4
    ACCELDATARATE_1600HZ = 0b1010 << 4

    MAGGAIN_2GAUSS = 0b00 << 5
    MAGGAIN_4GAUSS = 0b01 << 5
    MAGGAIN_8GAUSS = 0b10 << 5
    MAGGAIN_12GAUSS = 0b11 << 5

    MAGDATARATE_3_125HZ = 0b000 << 2
    MAGDATARATE_6_25HZ = 0b001 << 2
    MAGDATARATE_12_5HZ = 0b010 << 2
    MAGDATARATE_25HZ = 0b011 << 2
    MAGDATARATE_50HZ = 0b100 << 2
    MAGDATARATE_100HZ = 0b101 << 2

    GYROSCALE_245DPS = 0b00 << 4
    GYROSCALE_500DPS = 0b01 << 4
    GYROSCALE_2000DPS = 0b10 << 4

    def __init__(self, gpio_interrupt_num=161, i2c_bus_num=0, fifo_size=4):

        self._pin_int_gpio_num = gpio_interrupt_num
        self._i2c_bus_num = i2c_bus_num
        self._fifo_size = fifo_size

        # Hardware Resources
        self._pin_int = None
        self._smbus = None

        # Async Resources
        self._thread = Thread(target=self._main_loop)
        self._shutdown_event = Event()

    def start(self):
        self._thread.start()

    def shutdown(self):
        self._shutdown_event.set()

    def _init_hardware(self):
        # Initialize Hardware
        self._pin_int = GPIO(self._pin_int_gpio_num, 'in', 'rising')
        self._smbus = SMBus(self._i2c_bus_num)

    def _detect_who_am_i(self):
        who_am_i_accelmag = self._i2c_read_byte(LSM9DS0.XM_ADDRESS, LSM9DS0.WHO_AM_I_G)
        if who_am_i_accelmag != 0b01001001:
            return False

        who_am_i_gyro = self._i2c_read_byte(LSM9DS0.G_ADDRESS, LSM9DS0.WHO_AM_I_G)
        if who_am_i_gyro != 0b11010100:
            return False

        return True

    def _i2c_write_byte(self, address, register, value):
        self._smbus.write_byte_data(address, register, value)

    def _i2c_read_byte(self, address, register):
        return self._smbus.read_byte_data(address, register)

    def _enable_fifo_irq(self):
        # 11111 = 32 fifo size, not 100000
        fifo_bitmask = self._fifo_size - 1

        self._i2c_write_byte(LSM9DS0.G_ADDRESS, LSM9DS0.FIFO_CTRL_REG, 0b00100000 | fifo_bitmask)

    def _disable_fifo_irq(self):
        self._i2c_write_byte(LSM9DS0.G_ADDRESS, LSM9DS0.FIFO_CTRL_REG, 0b00000000)

    def _init_registers(self):

        self._disable_fifo_irq()

        # --Initialize Magnometer / Accelerometer--
        # Enable FIFO
        self._i2c_write_byte(LSM9DS0.XM_ADDRESS, LSM9DS0.CTRL_REG0_XM, 0b01000000)

        # 100 Hz
        # Enable X Y Z
        self._i2c_write_byte(LSM9DS0.XM_ADDRESS, LSM9DS0.CTRL_REG1_XM, 0b01100111)

        # 773 hz anti-alias filter
        # +/- 16 g
        self._i2c_write_byte(LSM9DS0.XM_ADDRESS, LSM9DS0.CTRL_REG2_XM, 0b00100000)

        # High resolution
        # 100 Hz
        self._i2c_write_byte(LSM9DS0.XM_ADDRESS, LSM9DS0.CTRL_REG5_XM, 0b01110100)

        # +/- 12 gauss
        self._i2c_write_byte(LSM9DS0.XM_ADDRESS, LSM9DS0.CTRL_REG6_XM, 0b01100000)

        # Continuous
        self._i2c_write_byte(LSM9DS0.XM_ADDRESS, LSM9DS0.CTRL_REG7_XM, 0b00000000)

        # --Initialize Gyrometer--
        # Disable Power Down Mode
        # Enable X Y Z
        self._i2c_write_byte(LSM9DS0.G_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b00001111)

        # Enable FIFO watermark interupt on DRDY
        self._i2c_write_byte(LSM9DS0.G_ADDRESS, LSM9DS0.CTRL_REG3_G, 0b00000100)

        # FIFO Mode
        self._i2c_write_byte(LSM9DS0.G_ADDRESS, LSM9DS0.CTRL_REG5_G, 0b01000000)

        self._enable_fifo_irq()

    def _read_fifo(self):
        # Mag
        mag_data = []
        for i in range(0, self._fifo_size):
            mag_data.append(self._smbus.read_i2c_block_data(LSM9DS0.XM_ADDRESS, LSM9DS0.OUT_X_L_M, 6))

        # Accel
        mag_data = []
        for i in range(0, self._fifo_size):
            mag_data.append(self._smbus.read_i2c_block_data(LSM9DS0.XM_ADDRESS, LSM9DS0.OUT_X_L_A, 6))

        # Gyro
        mag_data = []
        for i in range(0, self._fifo_size):
            mag_data.append(self._smbus.read_i2c_block_data(LSM9DS0.G_ADDRESS, LSM9DS0.OUT_X_L_G, 6))

    def _main_loop(self):

        self._init_hardware()

        if not self._detect_who_am_i():
            raise ValueError("Could not detect I2C device")

        self._init_registers()

        while not self._shutdown_event.is_set():
            event = self._pin_int.wait_for_int(timeout=0.1)
            if len(event) == 1 and event[0][1] == 10:
                self._read_fifo()

                # Clear Interrupt
                self._disable_fifo_irq()
                self._enable_fifo_irq()
