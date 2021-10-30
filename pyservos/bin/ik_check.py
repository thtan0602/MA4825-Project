import threading
from pyservos.servo_serial import ServoSerial
from pyservos.ax12 import AX12
from pyservos.utils import angle2int, le
import time


import rospy
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox

from math import *
import numpy as numpy

a1 = 6.7
a2 = 4.5
a3= 9.5
a4 = 9.5
a5 = 16.5
offset_angle = 0
offset_height = a1 + a2

PI=pi

constants=numpy.zeros(7)
constants[0]=a3
constants[1]=a4
constants[2]=a5
constants[3]=offset_angle
constants[4]=offset_height
constants[5]=a1
constants[6]=a2

offset = [60,150,150,240]

CONVEYOR_ID = 20
BASE_ID = 0
LINK1_ID = 2
LINK2_ID = 3
LINK3_ID = 1

def rad2deg(angle):
    return angle*180/PI

def deg2rad(angle):
    return angle*PI/180

def IK(target_pose,constants):   #target position xyz and constants for the calculation
    x = target_pose[0]
    y = target_pose[1]
    z = target_pose[2]

    a3= float(constants[0])
    a4 = float(constants[1])
    a5 = float(constants[2])
    offset_angle = float(constants[3])
    offset_height = float(constants[4])
    #check if target position is valid
        #check radius of working radius
    maxR = a3 + a4 + a5
    r = sqrt(pow(x,2) + pow(y,2))
    if r > maxR:
        print("Invalid target pose")
    else:
        #else, find tetha 1, solve in x-y plane.
            #for axis
        if y==0 and x>0:  #forward
            deg1 = 0
        elif y==0 and x<0:  #backward
            deg1 = PI
        elif y>0 and x==0:   #left
            deg1 = PI/2
        elif y<0 and x==0:   #right
            deg1 = -PI
            #for default
        elif x==0 and y==0:   #if empty set default look forward
            deg1 = 0
            #for quads     NOTE:adjust the final value based on the limit of dynamixel motor
        elif x>0 and y>0:   #for 1st quad
            deg1 = atan(y/x)
        elif x<0 and y>0:  #for 2nd quad
            deg1 = PI + atan(y/x)
        elif x<0 and y<0:  #for 3rd quad
            deg1 = PI + atan(y/x)
        elif x>0 and y<0:  #for 4th quad
            deg1 = atan(y/x)

        #find tetha 2,3,4, solve in z-r plane
            # Desired Position in r-z plane
        r_ = r
        z_ = z - offset_height #height
        phi = offset_angle
        phi = deg2rad(phi)

        wx = r_ - a5*cos(phi)
        wy = z_ - a5*sin(phi)

        delta = wx**2 + wy**2
        c2 = ( delta -a3**2 -a4**2)/(2*a3*a4)
        s2 = sqrt(1-c2**2)
        deg3 = numpy.arctan2(s2, c2)

        s1 = ((a3+a4*c2)*wy - a4*s2*wx)/delta
        c1 = ((a3+a4*c2)*wx + a4*s2*wy)/delta
        deg2 = numpy.arctan2(s1,c1)
        deg4 = phi-deg2-deg3

        output = numpy.zeros((4))
        output[0] = rad2deg(deg1) + offset[0]
        output[1] = rad2deg(deg2) + offset[1]
        output[2] = rad2deg(deg3) + offset[2]
        output[3] = rad2deg(deg4) + offset[3]
        print([rad2deg(deg1),rad2deg(deg2),rad2deg(deg3),rad2deg(deg4)])
        return output

def moveTo(serial, ax, id, angle):
    angle_ = int(angle/300.0*1023)
    pkt = ax.makeServoMovePacket(id, angle)
    ret = serial.sendPkt(pkt)

    while (True):
        pkt = ax.makeReadAnglePacket(id)
        ret = serial.sendPkt(pkt)
        num = ret[-2]*255 + ret[-3]
        if (abs(num-angle_) <= 5):
            break
        time.sleep(0.5)

class Conveyor():
    MOVE_SPEED = le(200)
    STOP_SPEED = le(0)

    def __init__(self, ax, serial, id):
        self.thread = threading.Thread(target=self.run)
        self.event = threading.Event()
        self.thread.daemon = True
        self.state = True
        self.id = id
        self.ax = ax
        self.serial = serial

    def run(self):
        while True:
            self.action()
            self.event.wait()
            self.event.clear()
            self.state = not self.state

    def action(self):
        if self.state == True:
            print("Moving conveyor belt. ")
            pkt = self.ax.makeWritePacket(self.id, AX12.GOAL_VELOCITY, self.MOVE_SPEED)
            ret = self.serial.sendPkt(pkt)
        else:
            print("Stopping conveyor belt. ")
            pkt = self.ax.makeWritePacket(self.id, AX12.GOAL_VELOCITY, self.STOP_SPEED)
            ret = self.serial.sendPkt(pkt)

class RobotArm():
    def __init__(self):
        self.thread = threading.Thread(target=self.run)
        self.event = threading.Event()
        self.kb_event = threading.Event()
        self.thread.daemon = True

    def run(self):
        print("System Ready. Starting System. ")
        self.camera.thread.start()
        self.conveyor.thread.start()
        while True:
            print("Waiting for Object to be in place (Robot Arm). ")
            self.event.wait()
            self.event.clear()
            print("Object is in place. ")
            print("Moving Robot Arm. ")
            """
            Note: Remove the kb_event
            1) Pick up Object.
            2) Move to Box.
            3) Drop in Box.
            4) Move back to Ready Position.
            """
            moveTo(serial, ax, BASE_ID, 240)
            time.sleep(1.0)
            moveTo(serial, ax, BASE_ID, 150)
            time.sleep(1.0)
            # self.kb_event.wait()
            # self.kb_event.clear()
            print("Robot Arm Ready. ")
            self.camera.event.set()
            self.conveyor.event.set()

    def set_conveyor(self, conveyor):
        self.conveyor = conveyor

    def set_camera(self, camera):
        self.camera = camera

class Camera():
    def __init__(self):
        self.thread = threading.Thread(target=self.run)
        self.event = threading.Event()
        self.kb_event = threading.Event()
        self.thread.daemon = True

    def run(self):
        while True:
            print("Waiting for Object to be in place (Camera). ")
            """
            Note: Remove the kb_event
            Loop:
                1) Obtain image from ros. Use rospy.wait_for_message
                2) Perform Object Detection
                3) Check if object crosses threshold
                4) If True, break
                5) Else sleep

            Use global variable to define type of object to pick up.
            Use global variable to define position of object.
            Look at threading.Lock for proper synchronistion between threads.
            """
            # while True:
            #     data = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)
            #     coordinate,name = coord_callback(data)
            #     if
            self.kb_event.wait()
            self.kb_event.clear()

            self.conveyor.event.set()
            self.robot_arm.event.set()

            print("Waiting for Object Gripping to finish. ")
            self.event.wait()
            self.event.clear()


    def set_conveyor(self, conveyor):
        self.conveyor = conveyor

    def set_robot_arm(self, robot_arm):
        self.robot_arm = robot_arm


if __name__ == "__main__":
    #rospy.init_node('manipulation', anonymous = True)
    serial = ServoSerial('/dev/ttyUSB0')  # tell it what port you want to use
    serial.open()

    ax = AX12()

    # Set motor to wheel mode
    pkt = ax.makeWritePacket(CONVEYOR_ID, AX12.CW_ANGLE_LIMIT, angle2int(0))
    ret = serial.sendPkt(pkt)
    pkt = ax.makeWritePacket(CONVEYOR_ID, AX12.CCW_ANGLE_LIMIT, angle2int(0))
    ret = serial.sendPkt(pkt)

    # Change movement speed
    for i in [BASE_ID, LINK1_ID, LINK2_ID, LINK3_ID]:
        pkt = ax.makeWritePacket(i, AX12.GOAL_VELOCITY, le(200))
        ret = serial.sendPkt(pkt)
        pkt = ax.makeWritePacket(i, AX12.CW_ANGLE_LIMIT, angle2int(0))
        ret = serial.sendPkt(pkt)
        pkt = ax.makeWritePacket(i, AX12.CCW_ANGLE_LIMIT, angle2int(300))
        ret = serial.sendPkt(pkt)

    #ik calculation TEST
    #target_pos = IK([0,a4+a5-0.6,a1+a2+a3-0.1],constants)
    target_pos = IK([a4+a5-0.1,0,a1+a2+a3-0.1],constants)
    print(target_pos)



    # for id, position in zip([BASE_ID, LINK1_ID, LINK2_ID, LINK3_ID], [150, 150, 240, 240]):
    #     pkt = ax.makeServoMovePacket(id, position)
    pkt = ax.makeSyncMovePacket([[id, position] for id, position in zip([BASE_ID, LINK1_ID, LINK2_ID, LINK3_ID], target_pos)])
    ret = serial.sendPkt(pkt)
    time.sleep(2.0)

    conveyor = Conveyor(ax, serial,  CONVEYOR_ID)
    robot_arm = RobotArm()
    camera = Camera()

    robot_arm.set_conveyor(conveyor)
    robot_arm.set_camera(camera)
    camera.set_robot_arm(robot_arm)
    camera.set_conveyor(conveyor)

    robot_arm.thread.start()

    """
    Remove these in the real code.
    Just rospy.spin()
    """
    def myhook():
        pkt = ax.makeWritePacket(CONVEYOR_ID, AX12.GOAL_VELOCITY, Conveyor.STOP_SPEED)
        ret = serial.sendPkt(pkt)
        for id, position in zip([BASE_ID, LINK1_ID, LINK2_ID, LINK3_ID], [150, 150, 240, 240]):
            ax.makeServoMovePacket(id, position)
            ret = serial.sendPkt(pkt)

    #rospy.on_shutdown(myhook)
    #rospy.spin()

    while True:
         try:
             input("1: Object in reachable area? \n")
             camera.kb_event.set()
         except KeyboardInterrupt:
             pkt = ax.makeWritePacket(CONVEYOR_ID, AX12.GOAL_VELOCITY, Conveyor.STOP_SPEED)
             ret = serial.sendPkt(pkt)
             for id, position in zip([BASE_ID, LINK1_ID, LINK2_ID, LINK3_ID], [150, 150, 240, 240]):
                 ax.makeServoMovePacket(id, position)
                 ret = serial.sendPkt(pkt)
             exit()
