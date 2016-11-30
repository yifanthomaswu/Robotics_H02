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
motorParams.maxRotationAcceleration = 5.0
motorParams.maxRotationSpeed = 6.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
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

class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print "drawLine:" + str((x1,y1,x2,y2))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);
            
# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 10;    
        self.data = [];

    def update(self):
        self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)];
    
    def draw(self):
        canvas.drawParticles(self.data);

            
canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();

particles = Particles();

error = 0.2
varX = 2.1345**2/80
varY = 2.1786**2/80  

def calcM(x, y, t, Ax, Ay, Bx, By):
    top = (By - Ay) * (Ax - x) - (Bx - Ax) * (Ay - y)
    bottom = (By - Ay) * math.cos(t) - (Bx - Ax) * math.sin(t)
    return float(top)/bottom

def withinRange(x, y, Ax, Ay, Bx, By):
    if (abs(x-Ax) < 0.1):
        return ((y < max(Ay, By)) and (y > min(Ay, By)))
    elif (abs(y-Ay) < 0.1):
        return ((x < max(Ax, Bx)) and (x > min(Ax, Bx)))
    else:
        return False
    
def getWall(x, y, theta):
    global mymap
    wall = None
    for (Ax, Ay, Bx, By) in mymap.walls:
        m = calcM(x, y, theta, Ax, Ay, Bx, By)
        if (m < 0):
            continue
        interX, interY = x + m * math.cos(theta), y + m * math.sin(theta)
        #print "m= ", m, "with rage ", withinRange(interX, interY, Ax, Ay, Bx, By)
        if (withinRange(interX, interY, Ax, Ay, Bx, By) and (wall is None or wall[1] > m)):
            wall = ((Ax, Ay, Bx, By), m)
    return wall



#sd = 0.48305
sd = 2.0
K = 0.0000000001

def calculateLikelihood(x, y, theta, z):
    wall = getWall(x, y, theta)
    m = wall[1]
    #print "top= ", float(-((z-m)**2))
    likelihood = math.exp(float(-((z-m)**2)) / (2*(sd**2)))
    #print "dist to wall= ", m, "z= ", z, "like= ", likelihood

    #print "like= ", likelihood
    return likelihood + K

def move(angle):
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        continue
    return

def moveD(distance):
    #angle = distance * (3.65 / 10)
    angle = distance * (2.9 / 10)
    move(angle)
    updateCloudD(distance)
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
    updateCloudT(angle)
    angle = math.degrees(angle) * (3.45 / 90)
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return

def navigateToWaypoint(X, Y, flag):
    (x, y, t) = currentPosition()
    print (x, y, math.degrees(t))
    dx, dy = X - x, Y - y
    a = math.atan2(dy, dx)
    b = a - t
    if (b <= -math.pi):
        b += 2 * math.pi
    elif (b > math.pi):
        b -= 2 * math.pi
    turn(b)
    stop(motors)
    print "degree: " , math.degrees(b)
    particles.draw()
    time.sleep(1)
    if flag:
        MCL()
    (x, y, t) = currentPosition()
    print (x, y, t)
    dx, dy = X - x, Y - y
    d = math.sqrt(dx**2 + dy**2)
    if (d < 20):
        moveD(d)
        stop(motors)
        particles.draw()
        if flag:
            MCL()
    else:
        D = 20
        moveD(D)
        stop(motors)
        particles.draw()
        time.sleep(1)
        if flag:
            MCL()
        navigateToWaypoint(X, Y, flag)
    return

def currentPosition():
    global particles
    xBar = sum([(x * w) for x, _, _, w in particles.data])
    yBar = sum([y * w for (_, y, _, w) in particles.data])
    tBar = sum([t * w for (_, _, t, w) in particles.data])
    return (xBar, yBar, tBar)

def stop(motorS):
    if (len(motorS) > 1):
        interface.setMotorRotationSpeedReferences(motorS,[0.01,0.01])
        interface.setMotorPwm(motorS[0],0)
        interface.setMotorPwm(motorS[1],0)
    else:
        interface.setMotorRotationSpeedReferences(motorS,[0.01])
        interface.setMotorPwm(motorS[0],0)
    return

def updateCloudD(D):
    global particles
    particles.data = [(x + (D + random.gauss(0, math.sqrt(varX * abs(D))))*math.cos(t), y + (D + random.gauss(0, math.sqrt(varY * abs(D))))*math.sin(t), t + random.gauss(0, 0.02), w) for (x, y, t, w) in particles.data]
    return

def updateCloudT(A):
    global particles
    particles.data = [(x, y, t + A + random.gauss(0, 0.05), w) for (x, y, t, w) in particles.data]
    return

def normalize():
    global particles
    sumP = sum([w for (_,_,_,w) in particles.data])
    particles.data = [(x, y, t, w/sumP) for (x, y, t, w) in particles.data]
    return

def resample():
    global particles
    parray = [0] * particles.n
    for i in range(0, particles.n):
        parray[i] = sum([w for (_,_,_,w) in particles.data[0:i + 1]])
    #print parray
    newp = [0] * particles.n
    for i in range(0, particles.n):
        w = random.uniform(0, 1)
        for j in range(0, particles.n):
            if (parray[j] > w):
                newp[i] = particles.data[j]
                break
    particles.data = newp
    #Set all w to 1/N
    particles.data = [(x, y, t, float(1) / particles.n) for (x, y, t, _) in particles.data]
    return

def MCL():
    global particles
    z = readSonar()
    particles.data = [(x, y, t, calculateLikelihood(x, y, t, z)) for (x, y, t, w) in particles.data]
    normalize()
    resample()

    #interface.setMotorRotationSpeedReferences([motor], [speed])
#while not interface.motorRotationSpeedReferenceReached(motor):
#    time.sleep(0.1)

def scan():
    n = 2
    start_angle = interface.getMotorAngles([motor])[0][0]
    angle_range = math.pi * 0.85
    right = -1
    readings = []
    count = 0
    interface.increaseMotorAngleReference(motor, angle_range)
    while True:
        if abs(interface.getMotorAngleReferences([motor])[0] - interface.getMotorAngle(motor)[0]) < 0.05 :
            stop([motor])
            count += 1
            time.sleep(0.5)
            start_angle=interface.getMotorAngles([motor])[0][0]
            if count >= 2:
                break
            interface.increaseMotorAngleReference(motor, right*angle_range)
            right = right * -1
        if right == -1:
            reading = readSonar()
            new_angle = interface.getMotorAngles([motor])[0][0]
            a = (angle_range / 2) - abs(new_angle-start_angle)
            a *= right
            #print "a= ", a
            readings.append((reading, a))
            x = currentPosition()[0]
            y = currentPosition()[1]
            X = (math.cos(a + currentPosition()[2]) * reading) + x
            Y = (math.sin(a + currentPosition()[2]) * reading) + y
            canvas.drawLine((x, y, X, Y))
        time.sleep(0.01)
    
    return readings

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

def bump():
    #speed = 5
    #move(d)
    #interface.setMotorRotationSpeedReferences(motors,[speed,speed])
    distance = 35
    angle = distance * (2.9 / 10)
    while True:
        interface.increaseMotorAngleReferences(motors,[angle,angle])
        updateCloudD(distance)
        particles.draw()
        while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
            if (isTouchedL() or isTouchedR()):
                stop(motors)
                print "bumped"
                time.sleep(0.2)
                moveD(-6)
                return True
        time.sleep(0.2)
        return False
    
def goToBottle():    
    readings = scan()
    #mean = sum([getWall(currentPosition()[0], currentPosition()[1], a)[1] - d for (d, a) in readings]) / len(readings)
    #error_range = [getWall(currentPosition()[0], currentPosition()[1], a)[1] for (d, a) in readings]
    error_range = filter(lambda (d,a): (getWall(currentPosition()[0], currentPosition()[1], a + currentPosition()[2])[1] > d) and (getWall(currentPosition()[0], currentPosition()[1], a + currentPosition()[2])[1] - d > 10), readings)
    print "range is ", len(error_range)
    d, a = error_range[len(error_range) // 2]
    angle = a
    
    x = currentPosition()[0]
    y = currentPosition()[1]
    X = (math.cos(a + currentPosition()[2]) * 255) + x
    Y = (math.sin(a + currentPosition()[2]) * 255) + y
    canvas.drawLine((x+0.01, y, X+0.01, Y))
    canvas.drawLine((x+0.02, y, X+0.02, Y))
    canvas.drawLine((x+0.03, y, X+0.03, Y))
    canvas.drawLine((x+0.04, y, X+0.04, Y))

    
    
    print "angle is ", angle
    turn(angle)
    stop(motors)
    time.sleep(0.3)
    if bump():
        return
    else:
        goToBottle()
    return
    
def wayPointNav():
    global particles
    particles.draw()
    
    navigateToWaypoint(waypoints[1][0], waypoints[1][1], False)
    time.sleep(0.1)
    navigateToWaypoint(waypoints[2][0], waypoints[2][1], False)
    time.sleep(0.1)
    particles.draw()

    setSonar(1)
    goToBottle()
    setSonar(-1)
    
    navigateToWaypoint(waypoints[0][0], waypoints[0][1], True)
    time.sleep(0.1)
    navigateToWaypoint(waypoints[3][0], waypoints[3][1], True)
    time.sleep(0.1)
    navigateToWaypoint(waypoints[4][0], waypoints[4][1], False)
    time.sleep(0.1)
    particles.draw()

    setSonar(1)
    goToBottle()
    setSonar(-1)
    
    navigateToWaypoint(waypoints[5][0], waypoints[5][1], True)
    time.sleep(0.1)
    particles.draw()
    return

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
                time.sleep(0.3)
                return
            

def bumpStop(d):
    distance = d
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    while True:
        interface.increaseMotorAngleReferences(motors,[angle,angle])
        while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
            if (isTouchedL() or isTouchedR()):
                time.sleep(0.5)
                stop(motors)
                print "bumped"
                time.sleep(0.2)
                moveD(-8)
                stop(motors)
                time.sleep(0.3)
                return
            
def turnSonarRight(m):
    interface.increaseMotorAngleReference(motor, math.pi * -0.55 * m)
    while (compareS(interface.getMotorAngleReferences([motor]), interface.getMotorAngles([motor]), error)):
        continue
            
def driveBy():
    moveD(26)
    stop(motors)
    time.sleep(0.2)
    turn(math.pi * 0.48)
    objectOneFound = False
    turnSonarRight(1)
    time.sleep(1)
    
    distance = 80
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        reading = readSonar()
        print reading
        if (reading < 80):
            print "found1"
            time.sleep(0.2)
            moveD(15)
            stop(motors)
            time.sleep(0.2)
            turn(math.pi * -0.48) #right is negative
            stop(motors)
            time.sleep(0.3)
            moved = ((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9))
            bumpFaster(95)
            turn(math.pi * 0.48)
            stop(motors)
            distance = 80 - moved
            move(distance * (2.9 / 10))
            objectOneFound = True
            break
            
    print "second"
    time.sleep(0.5)
    
    distance = 90
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        reading = readSonar()
        print reading
        if (reading < 50):
            print "found2"
            time.sleep(0.2)
            moveD(15)
            stop(motors)
            time.sleep(0.2)
            turn(math.pi * -0.48) #right is negative
            stop(motors)
            time.sleep(0.3)
            moved = ((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9))
            bumpFaster(50)
            turn(math.pi * 0.48)
            time.sleep(0.3)
            move((moved + 16)  * (-2.9 / 10))
            stop(motors)
            break
                
        if (isTouchedL() or isTouchedR()):
            stop(motors)
            print "bumped"
            time.sleep(0.2)
            moveD(-((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9) + 16))
            stop(motors)
            break
            
    print "third"
    time.sleep(0.5)
    turn(math.pi * 0.48)
    
    distance = 90
    angle = distance * (2.9 / 10)
    initial = interface.getMotorAngles(motors)[0][0]
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        reading = readSonar()
        print reading
        if (reading < 90):
            print "found3"
            time.sleep(0.2)
            moveD(15)
            stop(motors)
            time.sleep(0.2)
            turn(math.pi * -0.48) #right is negative
            stop(motors)
            time.sleep(0.3)
            moved = ((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9))
            bumpFaster(100)
            turn(math.pi * 0.48)
            time.sleep(0.3)
            bumpStop(60)
            turn(math.pi * 0.48)
            time.sleep(0.3)
            turnSonarRight(-1)
            time.sleep(0.3)
            reading=readSonar()
            time.sleep(0.3)
            moveD(reading-30)
            time.sleep(0.3)
            turn(math.pi * 0.48)
            particles.data = [(10 ,30, 0, float(1) / particles.n)] * particles.n
            time.sleep(0.3)
            navigateToWayPoint(84,30)
            break
                
        if (isTouchedL() or isTouchedR()):
            stop(motors)
            print "bumped"
            time.sleep(0.2)
            moveD(-((interface.getMotorAngles(motors)[0][0] - initial) * (10/2.9) - 26))
            time.sleep(0.3)
            turn(math.pi * 0.48)
            moveD(34)
            break
    if(not objectOneFound):
        stop(motors)
        time.sleep(0.3)
        bumpFaster(120)

waypoints = [(84 ,30),
             (110, 42),
             (113, 42),
             (126, 80),
             (126, 83),
             (84, 30)]

particles.n = 100
#particles.data = [(waypoints[0][0], waypoints[0][1], 0, float(1) / particles.n)] * particles.n
particles.data = [(84 ,30, 0, float(1) / particles.n)] * particles.n
driveBy()
#turn(math.pi * 0.5)
#turnSonarRight(1)
#wayPointNav()
#goToBottle()
#setSonar(1)
#scan()
#setSonar(-1)
#particles.data = [(0, 0, 0, float(1) / particles.n)] * particles.n
#navigateToWaypoint(10, 0)
#navigateToWaypoint(10, 10)
interface.terminate()

