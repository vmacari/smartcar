import serial
import struct as structures
from multiprocessing import Process, Lock
import time
from enum import Enum
from utils.logger import loge, logi

class ComDriver:

    class Commands(Enum):
        cmdNone = 0
        cmdGetInstantSpeed = 1
        cmdGetAvgSpeed = 2
        cmdSetSpeed = 3
        cmdSetDirection = 4
        cmdHeartbeat = 5
        cmdAck = 6
        cmdNack = 7

    # self.data_received_callback(rx_buffer, byte_index)
    def __init__(self, port, baudrate):

        self.serial_port = serial.serial_for_url(port, timeout=1, do_not_open=1)

        self.serial_port.baudrate = baudrate
        self.serial_port.write_timeout = 1.0
        self.serial_port.read_timeout = 1.0

        self.serial_port.open()
        self.lock = Lock()

        self.p = Process(target=self.read_packet_data())
        self.p.start()

    def read_packet_data(self):

        byte_index = 0
        rx_buffer = list()

        while self.serial_port._isOpen:

            try:
                if self.serial_port.inWaiting() <= 0:
                    time.sleep(0.1)
                    continue

                b = self.serial_port.read()

                if byte_index == 0:
                    if b == '\xca':
                        byte_index = 1
                elif byte_index == 1:
                    if b == '\xfe':
                        byte_index = 2

                elif byte_index < 5:
                    rx_buffer.append(ord(b))
                    byte_index += 1

                    if byte_index == 5:
                        if self.data_received_callback is not None:
                            self.data_received_callback(rx_buffer, byte_index)

                        rx_buffer = list()
                        byte_index = 0

                else:
                    byte_index = 0

            except IOError as io_error:
                loge('IO Serial port exception : {0}'.format(io_error))
                break
            except serial.SerialException as se:
                loge('Serial port exception : {0}'.format(se))
                return

        return None

    def data_received_callback(self, data, count):
        raise NotImplemented('Method shoul be implemented')

    # [0xCAFE][CMD][MOTOR][DATA]
    def write_packet_data(self, cmd, motor, byte_data):

        self.lock.acquire()
        bb = bytearray()
        bb += structures.pack('B', 0xCA)
        bb += structures.pack('B', 0xFE)
        bb += structures.pack('B', cmd)
        bb += structures.pack('B', motor)
        bb += structures.pack('B', byte_data)

        self.serial_port.write(bb)
        self.lock.release()

        # Read back AK
        return True

class MotorBoard(ComDriver):

    def __init__(self):
        ComDriver.__init__(self, "/dev/ttyUSB0", 115200)


    def data_received_callback(self, data, count):
        #print ('Received {0}'.format(ComDriver.Commands(int(data[0]))))
        cmd = data[0]

        #print ('Received {0}'.format(ComDriver.Commands(cmd)))

        if ComDriver.Commands(cmd) is ComDriver.Commands.cmdHeartbeat:

            hbL = data[1]
            hbH = data[2]
            logi('Heartbeat {0}'.format(hbL | (hbH << 8)))

        if ComDriver.Commands(cmd) is ComDriver.Commands.cmdAck:
            pass

        if ComDriver.Commands(cmd) is ComDriver.Commands.cmdGetAvgSpeed:
            pass

        if ComDriver.Commands(cmd) is ComDriver.Commands.cmdGetInstantSpeed:
            pass

    def set_speed(self):
        pass



if __name__ == "__main__":

    # cd = ComDriver("/dev/ttyUSB0", 115200,
    #                data_received_callback=data_received_callback)
    #
    # print("Done")


    MotorBoard()

    while 1 == 1:
        print('.')
        time.sleep(1)



