#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles

import time
import random
import math
import brickpi

interface=brickpi.Interface()
interface.initialize()

motors = [3,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 20.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 400.0
motorParams.pidParameters.k_i = 100.0
motorParams.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)); # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)); # in cm

def calcW():
    return random.random();

def calcTheta():
    return random.randint(0,360);

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
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

port = 2;
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

error = 0.2
varX = 2.1345**2/50
varY = 2.1786**2/50


#while True:
#    usReading = interface.getSensorValue(port)
#    print usReading

#t = 0;
#while True:
#    particles.update();
#    particles.draw();
#    t += 0.05;
#    time.sleep(0.05);    

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

def turn(angle):
    updateCloudT(angle)
    angle = math.degrees(angle) * (3.45 / 90)
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return

def move20():
    move(10)
    #stop()
    time.sleep(1)
    return

def stop():
    interface.setMotorRotationSpeedReferences(motors,[0.01,0.01])
    interface.setMotorPwm(motors[0],0)
    interface.setMotorPwm(motors[1],0)
    return

def updateCloudD(D):
    global particles
    particles.data = [(x + (D + random.gauss(0, math.sqrt(varX * D)))*math.cos(t), y + (D + random.gauss(0, math.sqrt(varY * D)))*math.sin(t), t + random.gauss(0, 0.02), w) for (x, y, t, w) in particles.data]
    return

def updateCloudT(A):
    global particles
    particles.data = [(x, y, t + A + random.gauss(0, 0.05), w) for (x, y, t, w) in particles.data]
    return

def currentPosition():
    global particles
    xBar = sum([(x * w) for x, _, _, w in particles.data])
    yBar = sum([y * w for (_, y, _, w) in particles.data])
    tBar = sum([t * w for (_, _, t, w) in particles.data])
    return (xBar, yBar, tBar)

def navigateToWaypoint(X, Y):
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
    stop()
    print "degree: " , math.degrees(b)
    particles.draw()
    time.sleep(1)
    MCL()
    (x, y, t) = currentPosition()
    print (x, y, t)
    dx, dy = X - x, Y - y
    d = math.sqrt(dx**2 + dy**2)
    if (d < 20):
        moveD(d)
        stop()
        particles.draw()
        MCL()
    else:
        D = 20
        moveD(D)
        stop()
        particles.draw()
        time.sleep(1)
        MCL()
        navigateToWaypoint(X, Y)
    return

def calibrate():
    turn(math.pi/2)
    stop()
    time.sleep(0.5)
    MCL()
    particles.draw()
    turn(-math.pi)
    stop()
    time.sleep(0.5)
    MCL()
    particles.draw()
    turn(math.pi/2)
    stop()
    time.sleep(0.5)
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

def readSonar():
    readings = []
    i = 0
    while (i < 10):
        r = interface.getSensorValue(port)[0]
        if (r == 255):
            i -= 1
        else:
            readings.append(r)
        i += 1
    #print(readings)
    return max(set(readings), key = readings.count) + 6

waypoints = [(84 ,30),
             (180, 30),
             (180, 54),
             (138, 54),
             (138, 168),
             (114, 168),
             (114, 84),
             (84, 84),
             (84, 30)]

def MCL():
    global particles
    z = readSonar()
    particles.data = [(x, y, t, calculateLikelihood(x, y, t, z)) for (x, y, t, w) in particles.data]
    normalize()
    resample()
    
def wayPointNav():
    global particles
    particles.data = [(waypoints[0][0], waypoints[0][1], 0, float(1) / particles.n)] * particles.n
    particles.draw()
    i = 0 
    for waypoint in waypoints[1:]:
        navigateToWaypoint(waypoint[0], waypoint[1])
        time.sleep(1)
        particles.draw()
    return
    
def MCLTesting():
    global particles
    particles.data = [(0, 10, 0, float(1) / particles.n)] * particles.n
    particles.draw()
    zP = 210
    while True:
        move20()
        zP -= 20
        updateCloudD(20)
        z = readSonar()
        print "sonar = ", zP
        particles.data = [(x, y, t, calculateLikelihood(x, y, t, zP)) for (x, y, t, w) in particles.data]
        normalize()
        resample()
        particles.draw()
        time.sleep(1)
        if (zP < 20):
            print "wall reached"
            break
    return

particles.n = 100
#wayPointNav()
smotor=[2]
interface.motorEnable(smotor[0])
interface.setMotorAngleControllerParameters(smotor[0],motorParams)

interface.setMotorAngleReference(2,200)
while not interface.motorAngleReferencesReached(smotor) :
    print interface.getMotorAngles(smotor)
    time.sleep(0.1)
