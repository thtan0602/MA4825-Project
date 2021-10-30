#! /usr/bin/python3.6

from math import *
import rospy
import time
import numpy as np
import threading
from pyservos.servo_serial import ServoSerial
from pyservos.ax12 import AX12
from pyservos.utils import angle2int, le
import serial

# ROS messages
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

# Constant variables
port = "/dev/ttyUSB0"
arduino_port = "/dev/ttyUSB2"
CONVEYOR_ID = 20
J1_ID = 0
J2_ID = 2
J3_ID = 3
J4_ID = 1
ID_lst = [J1_ID,J2_ID,J3_ID,J4_ID]
offset = [60,60,150,150]

a1 = 6.7
a2 = 4.5
a3 = 9.5
a4 = 9.5
a5 = 15
offset_angle = 15
offset_height = a1 + a2

constants=np.zeros(7)
constants[0]=a3
constants[1]=a4
constants[2]=a5
constants[3]=offset_angle
constants[4]=offset_height
constants[5]=a1
constants[6]=a2

#camera center of the image (0,0 is at the base of the robot)
center_y = 9
center_x = -26
coord = [0,0,0]
cap_height = 9.8
paper_ball_height = 9.5
pcoord = [-15,0,15]   #check ee

arduino = serial.Serial(port=arduino_port, baudrate=9600)

class RobotArm():
    MOVE_SPEED = le(50)
    CW_ANGLE_LIMIT = angle2int(0)
    CCW_ANGLE_LIMIT = angle2int(300)
    
    def __init__(self, ax, serial, id, parameters, readyEvent, gripEvent, camera):
        self.ax = ax
        self.serial = serial
        self.id = id
        self.parameters = parameters
        self.readyEvent = readyEvent
        self.gripEvent = gripEvent
        self.camera = camera
        self.HOME = [150, 150, 240, 240]
        self.STANDBY = [150, 150, 150, 240]
        
        for i in id:
            pkt = ax.makeWritePacket(i, AX12.GOAL_VELOCITY, RobotArm.MOVE_SPEED)
            ret = serial.sendPkt(pkt)
            pkt = ax.makeWritePacket(i, AX12.CW_ANGLE_LIMIT, RobotArm.CW_ANGLE_LIMIT)
            ret = serial.sendPkt(pkt)
            pkt = ax.makeWritePacket(i, AX12.CCW_ANGLE_LIMIT, RobotArm.CCW_ANGLE_LIMIT)
            ret = serial.sendPkt(pkt)   
        
        self.goStandby()
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()
    
    def run(self):
        while not rospy.is_shutdown():
            gripEvent.wait()
            gripEvent.clear()
            position = self.camera.getPosition()
            self.moveTo(position, gripper_first= True, turn_first=True)
            #self.openGripper()
            time.sleep(0.1)
            self.closeGripper()
            time.sleep(0.1)
            self.goHome()          # Replace this with box position (TODO)
            print("Go to Box")
            time.sleep(0.1)
            self.openGripper()
            time.sleep(0.1)
            #self.closeGripper()
            time.sleep(0.1)
            self.goStandby()
            print("Ready")
            self.signalReady()
    
    def IK(self, x, y, z):
        a3= float(self.parameters[0])
        a4 = float(self.parameters[1])
        a5 = float(self.parameters[2])
        offset_angle = float(self.parameters[3])
        offset_height = float(self.parameters[4])
        
        maxR = a3 + a4 + a5
        r = sqrt(pow(x,2) + pow(y,2))
        if r > maxR:
            print("Invalid target pose")
            return self.STANDBY
        else:
            deg1 = atan2(y, x)
            z_ = z - offset_height

            phi = offset_angle
            phi = radians(phi)

            wx = r - a5*cos(phi)
            wy = z_ - a5*sin(phi)

            delta = sqrt(wx**2 + wy**2)
            c2 = round(( -delta**2 +a3**2 +a4**2)/(2*a3*a4),5)
            s2 = sqrt(1-c2**2)
            alpha = atan2(s2,c2)
            deg3 = -(pi-alpha)
            psi = atan2(a4*sin(abs(deg3)),a3+a4*cos(deg3))
            beta = atan2(wy,wx)
            deg2 = psi + beta
            deg4 = phi-deg2-deg3

            output = list(map(degrees,[deg1, deg2, deg3, deg4]))
            output = list(map(lambda x, y: x + y, output, offset))
            output = list(map(lambda x: 300 - x, output))
            
            return output

    def signalReady(self):
        if not self.readyEvent.is_set():
            self.readyEvent.set()

    def moveTo(self, position, angle=False, gripper_first=False, turn_first=False):
        if angle:
            output = position
        else:
            output = self.IK(*position)
        if gripper_first:
            pkt = self.ax.makeSyncMovePacket([(self.id[3],output[3])])
            ret = self.serial.sendPkt(pkt)
            while not rospy.is_shutdown():
                pkt = self.ax.makeReadAnglePacket(self.id[3])
                ret = self.serial.sendPkt(pkt)
                angle = (ret[-2]*255 + ret[-3])/1023*300
                if (abs(angle - output[3]) < 5):
                    break
        if turn_first:
            pkt = self.ax.makeSyncMovePacket([(self.id[0],output[0])])
            ret = self.serial.sendPkt(pkt)
            while not rospy.is_shutdown():
                pkt = self.ax.makeReadAnglePacket(self.id[0])
                ret = self.serial.sendPkt(pkt)
                angle = (ret[-2]*255 + ret[-3])/1023*300
                if (abs(angle - output[0]) < 5):
                    break
        pkt = self.ax.makeSyncMovePacket([(id, angle) for id, angle in zip(self.id, output)])
        ret = self.serial.sendPkt(pkt)
        while not rospy.is_shutdown():
            for i in range(len(self.id)):
                pkt = self.ax.makeReadAnglePacket(self.id[i])
                ret = self.serial.sendPkt(pkt)
                angle = (ret[-2]*255 + ret[-3])/1023*300
                if (abs(angle - output[i]) > 5):
                    break
            else:
                return
            
            time.sleep(0.2)
    
    def openGripper(self):
        print("Open Gripper")
        # (TODO)
        arduino.write(b'H')
        time.sleep(2.0)

    def closeGripper(self):
        print("Close Gripper")
        # (TODO)
        arduino.write(b'H')
        time.sleep(2.0)
        
    def goHome(self):
        self.moveTo(self.HOME, angle=True)
    
    def goStandby(self):
        self.moveTo(self.HOME, angle=True)
 
        
class Camera():
    def __init__(self, inPlaceEvent, readyEvent):
        self.inPlaceEvent = inPlaceEvent
        self.readyEvent = readyEvent
        self.position = None
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout = 0.1)
                # self.position = [0, 23, 11]
                self.callback(data)
                #self.position = [0, 23, 11]
                if self.position is not None:
                    
                    # Position in place (TODO)
                    print(self.position[1])
                    if self.position[1] < center_y:
                        print("Object In Place")
                        self.inPlaceEvent.set()
                        self.inPlaceEvent.clear()
                        readyEvent.wait()
                        
            except rospy.ROSException:
                self.position = None
            
    
    def callback(self, data):
        bboxes = data.bounding_boxes
        bboxes.sort(key=lambda x: (x.xmin + x.xmax)/2, reverse=True)
        box = bboxes[0]
        # Process self.position (TODO)
        raw_x = (box.xmax + box.xmin)/2
        raw_y = (box.ymax + box.ymin)/2
        name = box.Class

        #shift 0,0 pixel to center
        tuned_x = raw_x - (640/2)
        tuned_y = -raw_y + (480/2)
        tuned_xy = [tuned_x,tuned_y]
        #print("PIXELc",tuned_xy)
        #scale to world
        #at max height:         view = (36x27)cm      image size = (640,480)
        x_scale = 36 / 640
        y_scale = 27 / 480

        real_x = tuned_x * x_scale
        real_y = tuned_y * y_scale
        #print("realCoordFrame,",[real_x,real_y])
        
        #transform the coord based on world frame
        coord[0] = real_y + center_x
        coord[1] = real_x + center_y

        if name == 'Bottle cap':
            coord[2] = cap_height
        elif name == 'Paper ball':
            coord[2] = paper_ball_height
        elif name == 'nil':
            print("No item has been detected yet.")
        self.position = coord

    def getPosition(self):
        
        #self.position = coord
        return self.position


class Conveyor():
    MOVE_SPEED = le(200)
    STOP_SPEED = le(0)
    
    def __init__(self, ax, serial, ID, readyEvent, inPlaceEvent, gripEvent):
        self.ax = ax
        self.serial = serial
        self.id = ID
        
        pkt = ax.makeWritePacket(ID, AX12.CW_ANGLE_LIMIT, angle2int(0))
        ret = serial.sendPkt(pkt)
        pkt = ax.makeWritePacket(ID, AX12.CCW_ANGLE_LIMIT, angle2int(0))
        ret = serial.sendPkt(pkt)
        
        self.readyEvent = readyEvent
        self.inPlaceEvent = inPlaceEvent
        self.gripEvent = gripEvent
        
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()
    
    def run(self):
        while not rospy.is_shutdown():
            readyEvent.wait()
            self.move()
            time.sleep(0.1)
            inPlaceEvent.wait()
            self.stop()
            time.sleep(0.1)
            gripEvent.set()
            
            readyEvent.clear()
    
    def move(self):
        pkt = self.ax.makeWritePacket(self.id, AX12.GOAL_VELOCITY, Conveyor.MOVE_SPEED)
        ret = self.serial.sendPkt(pkt)
        
    def stop(self):
        pkt = self.ax.makeWritePacket(self.id, AX12.GOAL_VELOCITY, Conveyor.STOP_SPEED)
        ret = self.serial.sendPkt(pkt)

if __name__ == "__main__":
    rospy.init_node('waste_sorter')
    
    ax = AX12()
    serial = ServoSerial(port)
    serial.open()
    
    inPlaceEvent = threading.Event()
    readyEvent = threading.Event()
    gripEvent = threading.Event()
    
    camera = Camera(inPlaceEvent, readyEvent)
    conveyor = Conveyor(ax, serial, CONVEYOR_ID, readyEvent, inPlaceEvent, gripEvent)
    robot_arm = RobotArm(ax, serial, ID_lst, constants, readyEvent, inPlaceEvent, camera)
    
    robot_arm.signalReady()
    
    def shutdown_handler():
        robot_arm.goHome()
        conveyor.stop()
        serial.close()

    rospy.on_shutdown(shutdown_handler)
    
    # Blocks until program ends
    rospy.spin()


    # try:
    #     while not rospy.is_shutdown():
    #         input("Press to signal object in place.")
    #         kb_event.set()
    #         time.sleep(0.5)
    #         kb_event.clear()
    # except KeyboardInterrupt:
    #     robot_arm.goHome()
    #     conveyor.stop()
    #     serial.close()
