from threading import Thread, Event
from sysfs_gpio import GPIO


class LSM9DS0(object):
    def __init__(self, gpio_interrupt_number=161):
        self._pin_int = None
        self._pin_int_gpio_num = gpio_interrupt_number

        self._thread = Thread(target=self._main_loop)
        self._shutdown_event = Event()

    def start(self):
        self._thread.start()

    def shutdown(self):
        self._shutdown_event.set()

    def _main_loop(self):
        self._pin_int = GPIO(self._pin_int_gpio_num, 'in', 'rising')

        while not self._shutdown_event.is_set():
            event = self._pin_int.wait_for_int(timeout=1.)
            if len(event) == 1 and event[0][1] == 10:
                # TODO: Read FIFO buffer
                pass


lsm9ds0 = LSM9DS0()
lsm9ds0.start()

