import select


class GPIO(object):
    def __init__(self, number, direction, edge=None):
        self.number = number
        self.direction = direction
        self.edge = edge

        self._sysfs_path = '/sys/class/gpio/gpio%d' % number

        try:
            self._value_file = open(self._sysfs_path + '/value', 'r+')
        except IOError as e:
            if e.errno == 2:
                self._export()
                self._value_file = open(self._sysfs_path + '/value', 'r+')
            else:
                raise e

        self.set_direction(direction)
        if direction == 'in' and edge is not None:
            self.set_edge(edge)

            self._poll_queue = select.epoll()
            self._poll_queue.register(self._value_file, select.EPOLLPRI | select.EPOLLET)

            # Eat the first event
            _ = self._poll_queue.poll()

    def _write_sysfs(self, path, value):
        sysfs_file = open(path, 'w')
        sysfs_file.write(value)
        sysfs_file.close()

    def _read_sysfs(self, path):
        sysfs_file = open(path, 'r')
        data = sysfs_file.read()
        sysfs_file.close()
        return data

    def set_direction(self, direction):
        self._write_sysfs(self._sysfs_path + '/direction', direction)
        self.direction = direction

    def get_direction(self):
        return self._read_sysfs(self._sysfs_path + '/direction')

    def set_edge(self, edge):
        self._write_sysfs(self._sysfs_path + '/edge', edge)
        self.edge = edge

    def get_edge(self):
        return self._read_sysfs(self._sysfs_path + '/edge')

    def get_value(self):
        return self._value_file.read()

    def set_value(self, value):
        self._value_file.write(value)

    def _export(self):
        self._write_sysfs('/sys/class/gpio/export', '%d' % self.number)

    def _unexport(self):
        self._write_sysfs('/sys/class/gpio/unexport', '%d' % self.number)

    def wait_for_int(self, timeout=-1.):
        if self.direction is not 'in' or self.edge is None:
            raise ValueError("Cannot wait for interrupt if pin is not in input mode or does not have an edge mode set")
        return self._poll_queue.poll(timeout=timeout)
