#wheel mode for continuous rotation of motor
from pyservos.servo_serial import ServoSerial
from pyservos.ax12 import AX12
from pyservos.utils import le

import time

serial = ServoSerial('COM6')  # tell it what port you want to use
# serial = ServoSerial('dummy')  # use a dummy serial interface for testing
serial.open()

ax = AX12()
cw_angle_limit_change = 0
val = le(cw_angle_limit_change)
pkt = ax.makeWritePacket(20, AX12.CW_ANGLE_LIMIT, val)
ret = serial.sendPkt(pkt)

ccw_angle_limit_change = 0
val = le(ccw_angle_limit_change)
pkt = ax.makeWritePacket(20, AX12.CCW_ANGLE_LIMIT, val)
ret = serial.sendPkt(pkt)
time.sleep(1)

speed = 1024
val = le(speed)
pkt = ax.makeWritePacket(20, AX12.GOAL_VELOCITY, val)   
ret = serial.sendPkt(pkt)


