import brickpi
import time
import math
import random

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

error = 0.2
NUMBER_OF_PARTICLES = 100
particles = [(0, 0, 0, float(0.01))] * NUMBER_OF_PARTICLES
offset = 50
scale = 15
sdX = math.sqrt(2.1345/16)
sdY = math.sqrt(2.1786/16)


def move(angle):
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        continue
    return

def moveD(distance):
    angle = distance * (4.95 / 10)
    move(angle)
    updateCloudD(distance)
    return

def compare(reference, current, error) :
    #return true if error < reference - current
    if ((abs(reference[0] - current[0][0]) > error) and (abs(reference[1] - current[1][0]) > error)):
        return True
    else:
        return False

def left90deg():
    angle = 5.69
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return

def turn(angle):
    updateCloudT(angle)
    angle = math.degrees(angle) * (5.69 / 90)
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return

def right90deg():
    angle = -5.69
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return

def move10():
    move(4.95)
    #stop()
    time.sleep(1)
    return

def stop():
    interface.setMotorRotationSpeedReferences(motors,[0.01,0.01])
    interface.setMotorPwm(motors[0],0)
    interface.setMotorPwm(motors[1],0)
    return

def move40():
    for i in range(1,5):
        move10()
        updateCloudD(10)
        printCloud()
        #time.sleep(1)
    return

def updateCloudD(D):
    global particles
    particles = [(x + (D + random.gauss(0, sdX))*math.cos(t), y + (D + random.gauss(0, sdX))*math.sin(t), t + random.gauss(0, 0.01), w) for (x, y, t, w) in particles]
    return

def updateCloudT(A):
    global particles
    particles = [(x, y, t + A + random.gauss(0, 0.01), w) for (x, y, t, w) in particles]
    return
    

def printSquare():
    #left
    print "drawLine:" + str((offset, offset, offset, offset + 40*scale))
    #right
    print "drawLine:" + str((offset + 40*scale, offset, offset + 40*scale, offset + 40*scale))
    #bottom
    print "drawLine:" + str((offset + 40*scale, offset + 40*scale, offset, offset + 40*scale))
    #top
    print "drawLine:" + str((offset, offset, offset + 40*scale, offset))
    return

def aSP(particles):
    return (offset + scale * particles[0], (offset + scale * 40) - scale * particles[1], particles[2])

def printCloud():
    global particles
    print "drawParticles:" + str([aSP(particle) for particle in particles])
    time.sleep(0.25)
    return

def currentPosition():
    global particles
    xBar = sum([(x * w) for x, _, _, w in particles])
    yBar = sum([y * w for (_, y, _, w) in particles])
    tBar = sum([t * w for (_, _, t, w) in particles])
    return (xBar, yBar, tBar)

def navigateToWaypoint(X, Y):
    (x, y, t) = currentPosition()
    dx, dy = X - x, Y - y
    a = math.atan2(dy, dx)
    b = a - t
    if (b <= -math.pi):
        b += 2 * math.pi
    elif (b > math.pi):
        b -= 2 * math.pi
    turn(b)
    stop()
    #time.sleep(0.1)
    d = math.sqrt(dx**2 + dy**2)
    moveD(d)
    
    
while (True):
    X = input("Enter X coordinate: ")
    Y = input("Enter Y coordinate: ")
    navigateToWaypoint(X*100, Y*100)
        
    
interface.terminate()
