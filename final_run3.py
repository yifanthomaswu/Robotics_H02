#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os

import time
import math
import brickpi

from operator import add

interface=brickpi.Interface()
interface.initialize()

#Sonar Motor
motor = 2
#Wheel motors
motors = [3,1]

interface.motorEnable(motor)
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

#Sonar port
sonar = 2
interface.sensorEnable(sonar, brickpi.SensorType.SENSOR_ULTRASONIC);

#Touch sensors
touch_portL = 0
touch_portR = 3
interface.sensorEnable(touch_portL, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touch_portR, brickpi.SensorType.SENSOR_TOUCH)

#Set motor params
motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 10.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 19.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 400
motorParams.pidParameters.k_i = 100
motorParams.pidParameters.k_d = 0


#Apply motor params to motors
interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 14.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 20.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 300.0
motorParams.pidParameters.k_i = 100.0
motorParams.pidParameters.k_d = 0.0
interface.setMotorAngleControllerParameters(motor,motorParams)
    
def readSonar():
    readings = []
    i = 0
    while (i < 10):
        r = interface.getSensorValue(sonar)[0]
        readings.append(r)
        print r
        i += 1
    return max(set(readings), key = readings.count)

error = 0.2
varX = 2.1345**2/80
varY = 2.1786**2/80  

#sd = 0.48305
sd = 2.0
K = 0.0000000001

def move(angle):
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        continue
    return

def moveD(distance):
    #angle = distance * (3.65 / 10)
    angle = distance * (2.9 / 10)
    move(angle)
    return

def compare(reference, current, error) :
    #return true if error < reference - current
    if ((abs(reference[0] - current[0][0]) > error) and (abs(reference[1] - current[1][0]) > error)):
        return True
    else:
        return False
    
def compareS(reference, current, error) :
    #return true if error < reference - current
    if (abs(reference[0] - current[0][0]) > error):
        return True
    else:
        return False

def turn(angle):
    angle = math.degrees(angle) * (3.45 / 90)
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return


def stop(motorS):
    if (len(motorS) > 1):
        interface.setMotorRotationSpeedReferences(motorS,[0.01,0.01])
        interface.setMotorPwm(motorS[0],0)
        interface.setMotorPwm(motorS[1],0)
    else:
        interface.setMotorRotationSpeedReferences(motorS,[0.01])
        interface.setMotorPwm(motorS[0],0)
    return


def setSonar(mul):
    angle = math.degrees(math.pi * 0.215) * (3.45 / 90) * mul
    interface.increaseMotorAngleReferences([motor],[-angle,angle])
    while (compareS(interface.getMotorAngleReferences([motor]), interface.getMotorAngles([motor]), 0.1)):
        continue
        
def isTouchedL():
    result = interface.getSensorValue(touch_portL)
    if result :
        touched = result[0]
        return touched
    else :
        print("Error reading sensorL")
        return -1
    
def isTouchedR():
    result = interface.getSensorValue(touch_portR)
    if result :
        touched = result[0]
        return touched
    else :
        print("Error reading sensorR")
        return -1


def bumpFaster(d):
    distance = d
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    while True:
        interface.increaseMotorAngleReferences(motors,[angle,angle])
        while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
            if (isTouchedL() or isTouchedR()):
                stop(motors)
                print "bumped"
                moveD(-((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9)))
                stop(motors)
                time.sleep(0.15)
                return
            

def bumpStop(d):
    distance = d
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    while True:
        interface.increaseMotorAngleReferences(motors,[angle,angle])
        while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
            if (isTouchedL() or isTouchedR()):
                moveD(10)
                stop(motors)
                print "bumped"
                time.sleep(0.15)
                moveD(-11)
                stop(motors)
                time.sleep(0.15)
                return
            
def turnSonarRight(m):
    interface.increaseMotorAngleReference(motor, math.pi * -0.55 * m)
    while (compareS(interface.getMotorAngleReferences([motor]), interface.getMotorAngles([motor]), error)):
        continue
            
def driveBy():
    moveD(28)
    stop(motors)
    time.sleep(0.15)
    turn(math.pi * 0.48)
    objectOneFound = False
    time.sleep(0.25)
    
    distance = 80
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        reading = readSonar()
        print reading
        if (reading < 78):
            print "found1"
            time.sleep(0.15)
            moveD(9)
            stop(motors)
            time.sleep(0.15)
            turn(math.pi * -0.48) #right is negative
            stop(motors)
            time.sleep(0.25)
            moved = ((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9))
            bumpFaster(85)
            moveD(-10)
            stop(motors)
            time.sleep(0.15)
            turn(math.pi * 0.48)
            stop(motors)
            time.sleep(0.25)
            distance = 80 - moved
            move(distance * (2.9 / 10))
            objectOneFound = True
            break
            
    print "second"
    time.sleep(0.15)
    
    distance = 90
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        reading = readSonar()
        print reading
        if (reading < 45):
            print "found2"
            time.sleep(0.15)
            moveD(8)
            stop(motors)
            time.sleep(0.15)
            turn(math.pi * -0.48) #right is negative
            stop(motors)
            time.sleep(0.25)
            moved = ((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9))
            bumpFaster(50)
            turn(math.pi * 0.48)
            time.sleep(0.25)
            move((moved + 16)  * (-2.9 / 10))
            stop(motors)
            break
                
        if (isTouchedL() or isTouchedR()):
            stop(motors)
            print "bumped"
            time.sleep(0.15)
            moveD(-((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9) + 16))
            stop(motors)
            break
            
    print "third"
    time.sleep(0.15)
    turn(math.pi * 0.48)
    stop(motors)
    time.sleep(0.25)
    
    distance = 95
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        reading = readSonar()
        print reading
        if (reading < 95):
            print "found3"
            time.sleep(0.15)
            moveD(8)
            stop(motors)
            time.sleep(0.15)
            turn(math.pi * -0.48) #right is negative
            stop(motors)
            time.sleep(0.25)
            moved = ((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9))
            bumpFaster(100)
            turn(math.pi * 0.48)
            time.sleep(0.15)
            bumpStop(60)
            turn(math.pi * 0.48)
            time.sleep(0.25)
            turnSonarRight(-1)
            time.sleep(0.15)
            reading=readSonar()
            time.sleep(0.15)
            moveD(reading-30)
            time.sleep(0.15)
            turn(math.pi * 0.48)
            stop(motors)
            time.sleep(0.25)
            moveD(65)
            break
                
        if (isTouchedL() or isTouchedR()):
            stop(motors)
            print "bumped"
            time.sleep(0.15)
            moveD(-((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9) - 26))
            time.sleep(0.15)
            turn(math.pi * 0.48)
            stop(motors)
            time.sleep(0.25)
            moveD(30)
            break
            
    if(not objectOneFound):
        stop(motors)
        time.sleep(0.15)
        bumpFaster(120)

driveBy()
interface.terminate()

