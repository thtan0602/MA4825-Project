from numpy import *
import math
import time

# Length of links in cm
a1= 10.45
a2 = 6.75
a3 = 16.5

# Desired Position of End effector
px = -23.25 # 23.25 or -23.25
py = 10.45

phi = 180
phi = deg2rad(phi)

# Equations for Inverse kinematics
wx = px - a3*cos(phi)
wy = py - a3*sin(phi)

delta = wx**2 + wy**2
c2 = ( delta -a1**2 -a2**2)/(2*a1*a2)

theta_2 = arccos(c2)
theta_1 = arctan2(wy, wx) - arctan2(a2*sin(theta_2), (a1+a2*cos(theta_2)))
theta_3 = phi-theta_1-theta_2

theta_1_deg = round(math.degrees(theta_1), 3) -90 + 150
theta_2_deg = round(math.degrees(theta_2), 3) + 150
theta_3_deg = round(math.degrees(theta_3), 3) + 150

print('theta_1: ',theta_1_deg)
print('theta_2: ', theta_2_deg)
print('theta_3: ', theta_3_deg)

# Run an AX-12 servo
from pyservos.servo_serial import ServoSerial
from pyservos.ax12 import AX12

serial = ServoSerial('COM6')  # tell it what port you want to use
# serial = ServoSerial('dummy')  # use a dummy serial interface for testing
serial.open()
# Move the robot arm
ax = AX12()
pkt0 = ax.makeServoMovePacket(0, 150) # move servo 1 to 150 degrees
ret0 = serial.sendPkt(pkt0)  # send packet, I don't do anything with the returned status packet

pkt1 = ax.makeServoMovePacket(1, theta_3_deg )  # move servo 1 to theta_3 degrees
ret1 = serial.sendPkt(pkt1)  # send packet, I don't do anything with the returned status packet

pkt2 = ax.makeServoMovePacket(2, theta_1_deg)  # move servo 1 to theta_1 degrees
ret2 = serial.sendPkt(pkt2)  # send packet, I don't do anything with the returned status packet

pkt3 = ax.makeServoMovePacket(3, theta_2_deg)  # move servo 1 to theta_2 degrees
ret3 = serial.sendPkt(pkt3)  # send packet, I don't do anything with the returned status packet

# facing the black face
# if (px == -23.25):
    # # Move the robot arm
    # ax = AX12()
    # pkt0 = ax.makeServoMovePacket(0, 150) # move servo 1 to 150 degrees
    # ret0 = serial.sendPkt(pkt0)  # send packet, I don't do anything with the returned status packet

    # pkt1 = ax.makeServoMovePacket(1, theta_3_deg )  # move servo 1 to theta_3 degrees
    # ret1 = serial.sendPkt(pkt1)  # send packet, I don't do anything with the returned status packet

    # pkt2 = ax.makeServoMovePacket(2, theta_1_deg)  # move servo 1 to theta_1 degrees
    # ret2 = serial.sendPkt(pkt2)  # send packet, I don't do anything with the returned status packet

    # pkt3 = ax.makeServoMovePacket(3, theta_2_deg)  # move servo 1 to theta_2 degrees
    # ret3 = serial.sendPkt(pkt3)  # send packet, I don't do anything with the returned status packet

# theta_1_deg = round(math.degrees(theta_1), 3) + 90 
# theta_2_deg = round(math.degrees(theta_2), 3) + 60
# theta_3_deg = round(math.degrees(theta_3), 3) + 225

# print('theta_1: ',theta_1_deg)
# print('theta_2: ', theta_2_deg)
# print('theta_3: ', theta_3_deg)

# #facing the outer face
# if (px == 23.25):

#     # Move the robot arm
#     ax = AX12()
#     pkt0 = ax.makeServoMovePacket(0, 0) # move servo 1 to 150 degrees
#     ret0 = serial.sendPkt(pkt0)  # send packet, I don't do anything with the returned status packet

#     pkt1 = ax.makeServoMovePacket(1, theta_3_deg )  # move servo 1 to theta_3 degrees
#     ret1 = serial.sendPkt(pkt1)  # send packet, I don't do anything with the returned status packet

#     pkt2 = ax.makeServoMovePacket(2, theta_1_deg)  # move servo 1 to theta_1 degrees
#     ret2 = serial.sendPkt(pkt2)  # send packet, I don't do anything with the returned status packet

#     pkt3 = ax.makeServoMovePacket(3, theta_2_deg)  # move servo 1 to theta_2 degrees







