from imaplib import Commands

import serial
import struct as structures
from multiprocessing import Process, Lock
import time
from enum import Enum
from utils.logger import loge, logi


class Commands(Enum):
    cmdNone = 0
    cmdGetInstantSpeed = 1
    cmdGetAvgSpeed = 2
    cmdSetSpeed = 3
    cmdSetDirection = 4
    cmdHeartbeat = 5
    cmdAck = 6
    cmdLog = 7
    cmdNack = 8


# class CommandType(Enum):
#     ctRead = 0
#     ctWrite = 1

class Motors(Enum):
    mLeftFront = 0
    mLeftRear = 1
    mRightFront = 2
    mRightRear = 3


class Directions(Enum):
    dirForward = 0
    dirBackward = 1



class ComDriver:

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

        cmd = 0
        motor = 0


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

                elif byte_index == 2:
                    cmd = b
                    byte_index = 3

                elif byte_index == 3:

                    if cmd == Commands.cmdLog:
                        len = b
                    else:
                        motor = b
                        len = 5

                    byte_index = 4


                elif byte_index < len:


                    if cmd != Commands.cmdLog:

                        self.data_received_callback(cmd, motor, b)

                        rx_buffer = list()
                        byte_index = 0
                        continue

                    if byte_index < len:
                        rx_buffer.append(ord(b))
                        byte_index += 1
                    else:
                        rx_buffer = list()

                        byte_index = 0
                        rx_buffer.append(0) # close string
                        self.log_message(rx_buffer)


                    # rx_buffer.append(ord(b))
                    # byte_index += 1
                    #
                    #
                    # if byte_index == 5:
                    #     if self.data_received_callback is not None:
                    #         self.data_received_callback(rx_buffer, byte_index)
                    #
                    #     rx_buffer = list()
                    #     byte_index = 0

                else:
                    byte_index = 0

            except IOError as io_error:
                loge('IO Serial port exception : {0}'.format(io_error))
                break
            except serial.SerialException as se:
                loge('Serial port exception : {0}'.format(se))
                return

        return None

    def log_message(self, message):
        logi('DEVICE :> {0}'.format(message))

    def data_received_callback(self, cmd, motor, data):
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

    def __init__(self, port ):
        ComDriver.__init__(self, port, 115200)

    def data_received_callback(self, cmd, motor, data):

        #print ('Received {0}'.format(ComDriver.Commands(int(data[0]))))
        #cmd = data[0]

        # print ('Received {0}'.format(ComDriver.Commands(cmd)))
        if Commands(cmd) is Commands.cmdHeartbeat:

            hbL = motor #data[1]
            hbH = data  #data[2]
            logi('Heartbeat {0}'.format(hbL | (hbH << 8)))

        if Commands(cmd) is Commands.cmdAck:
            pass

        if Commands(cmd) is Commands.cmdGetAvgSpeed:
            pass

        if Commands(cmd) is Commands.cmdGetInstantSpeed:
            pass

    # def set_speed(self):
    #     pass
    #
    # def set_direction(self):
    #     pass


    def move(self, dir, speed):

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mRightFront,
                                dir)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mLeftFront,
                                dir)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mRightRear,
                                dir)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mLeftRear,
                                dir)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mRightFront,
                                speed)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mLeftFront,
                                speed)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mRightRear,
                                Commands.Directions.dirForward)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mLeftRear,
                                Commands.Directions.dirForward)

    def move_forward(self, speed):
        self.move(Commands.Directions.dirForward, speed)

    def move_backward(self, speed):
        self.move(Commands.Directions.dirBackward, speed)

    def stop(self):
        self.move(Commands.Directions.dirForward, 0)


    def turn_left(self, speed):
        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mRightFront,
                                Commands.Directions.dirForward)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mLeftFront,
                                Commands.Directions.dirBackward)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mRightRear,
                                Commands.Directions.dirForward)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mLeftRear,
                                Commands.Directions.dirBackward)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mRightFront,
                                speed)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mLeftFront,
                                speed)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mRightRear,
                                Commands.Directions.dirForward)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mLeftRear,
                                Commands.Directions.dirForward)

    def turn_right(self, speed):

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mRightFront,
                                Commands.Directions.dirBackward)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mLeftFront,
                                Commands.Directions.dirForward)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mRightRear,
                                Commands.Directions.dirBackward)

        self.write_packet_data( Commands.cmdSetDirection,
                                Commands.Motors.mLeftRear,
                                Commands.Directions.dirForward)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mRightFront,
                                speed)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mLeftFront,
                                speed)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mRightRear,
                                Commands.Directions.dirForward)

        self.write_packet_data( Commands.cmdSetSpeed,
                                Commands.Motors.mLeftRear,
                                Commands.Directions.dirForward)

if __name__ == "__main__":

    # cd = ComDriver("/dev/ttyUSB0", 115200,
    #                data_received_callback=data_received_callback)
    #
    # print("Done")

    mb = MotorBoard(port = '/dev/cu.wchusbserial1420')

    while 1 == 1:
        print('.')
        mb.move_forward(50)
        time.sleep(1)



