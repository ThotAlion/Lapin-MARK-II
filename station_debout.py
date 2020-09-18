#!/usr/bin/python3
# -*- coding: utf-8 -*-

import pypot.robot
import time
import json
import math
import sys
import threading
import time
import queue
from serial import Serial
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def read_kbd_input(inputQueue):
    print('Ready for keyboard input:')
    while True:
        inputQueue.put(sys.stdin.read(1))

def interp(a, x1, x2):
    return x1+a*(x2-x1)


def interpInv(x, x1, x2):
    return (x-x1)/(x2-x1)


def MGD(theta2):
    c = math.cos(theta2)
    s = math.sin(theta2)
    xA = 0.0
    yA = 0.047
    xB = 0.094
    yB = 0.000
    L2 = 0.13350
    L3 = 0.10522
    L4 = 0.13269
    L5 = 0.13287
    xC = xB+L2*c
    yC = yB+L2*s
    AC = math.sqrt((xA-xC)**2+(yA-yC)**2)
    AH = min((L4**2-L3**2+AC**2)/(2*AC),L4)
    HD = math.sqrt(L4**2-AH**2)
    xH = xA+AH*(xC-xA)/AC
    yH = yA+AH*(yC-yA)/AC
    xD = xH-HD*(yC-yA)/AC
    yD = yH+HD*(xC-xA)/AC
    xF = xC+L5*(xC-xD)/L3
    yF = yC+L5*(yC-yD)/L3

    return math.atan((yF-yC)/(xF-xC))*180.0/math.pi, math.atan(yF/xF)*180.0/math.pi


lapin = pypot.robot.from_json('confLapinMarkIII.json')

PS = Serial('/dev/serial0',115200,timeout=0.1)
PS.flushInput()
info = {}

alpha = 0  # positif quand on ecarte
theta = 0  # negatif vers l'avant
aLc = 0  # repos à -40, extension à 30
aRc = 0  # repos à -40, extension à 30
ankleOffset = 5
compliant = True
speed = 100
state = 0
xLeft=0
xRight=0
KP = 10
KI = 5
rythme=1
srythme=10

inputQueue = queue.Queue()

inputThread = threading.Thread(target=read_kbd_input, args=(inputQueue,), daemon=True)
inputThread.start()
count = 0

last_update = time.time()

t0 = time.time()

while True:
    if (inputQueue.qsize() > 0):
        c = inputQueue.get()
        if c=='q':
            break
        if c=='a':
            state = -1

    # mesures
    # mesure de la temperature
    temp = 0
    for mot in lapin.motors:
        temp = max(temp, mot.present_temperature)
    if temp >60:
        print("HOT!")
    # mesure de l'angle quadrilatere
    aLm = interpInv(lapin.l_knee_y.present_position, -40, 30)
    aRm = interpInv(lapin.r_knee_y.present_position, -40, 30)
    # recuperation des capteurs
    PS.write(b"A")
    out = PS.readline()
    try:
        info = json.loads(out)
    except:
        pass

    print(str(temp)+'°C\t'+str(state))
    print(lapin.l_ankle_y.present_position)
    print(info)

    # machine a etat
    if state == 0:
        alpha = 10
        theta = 0
        aLc = 0.8
        aRc = 0.8
        speed = 5
        compliant = False

    elif state == -1:
        alpha = 0
        theta = 0
        aLc = 0.5
        aRc = 0.5
        speed = 10
        compliant = True


    # actionneurs
    (aFr,lFr) = MGD((90-lapin.r_knee_y.present_position)*math.pi/180.0)
    (aFl,lFl) = MGD((90-lapin.l_knee_y.present_position)*math.pi/180.0)
    lapin.r_hip_x.pid = (KP,KI,0)
    lapin.r_hip_x.compliant = compliant
    lapin.r_hip_x.goal_position = alpha/2
    lapin.r_hip_x.moving_speed = speed
    
    lapin.l_hip_x.pid = (KP,KI,0)
    lapin.l_hip_x.compliant = compliant
    lapin.l_hip_x.goal_position = alpha/2
    lapin.l_hip_x.moving_speed = speed
    
    lapin.r_hip_y.compliant = compliant
    lapin.r_hip_y.goal_position = 0-theta/2
    lapin.r_hip_y.moving_speed = speed
    
    lapin.l_hip_y.compliant = compliant
    lapin.l_hip_y.goal_position = 0+theta/2
    lapin.l_hip_y.moving_speed = speed
    
    lapin.r_knee_y.pid = (KP,KI,0)
    lapin.r_knee_y.compliant = compliant
    lapin.r_knee_y.goal_position = interp(aRc, -23, 90)
    lapin.r_knee_y.moving_speed = speed
    
    lapin.l_knee_y.pid = (KP,KI,0)
    lapin.l_knee_y.compliant = compliant
    lapin.l_knee_y.goal_position = interp(aLc, -23, 90)
    lapin.l_knee_y.moving_speed = speed
    
    lapin.r_ankle_y.compliant = compliant
    lapin.r_ankle_y.goal_position = aFr-lFr+ankleOffset
    lapin.r_ankle_y.moving_speed = speed
    
    lapin.l_ankle_y.compliant = compliant
    lapin.l_ankle_y.goal_position = aFl-lFl+ankleOffset
    lapin.l_ankle_y.moving_speed = speed

    # print("left")
    # print(lapin.l_knee_y.present_position)
    # print(lapin.l_ankle_y.goal_position)

    time.sleep(0.005)
for mot in lapin.motors:
    mot.compliant = True
time.sleep(0.04)
lapin.close()
