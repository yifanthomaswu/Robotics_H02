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
particles = [(0, 0, 0, 1/NUMBER_OF_PARTICLES)] * NUMBER_OF_PARTICLES
offset = 90
scale = 15
sdX = math.sqrt(2.1345/8)
sdY = math.sqrt(2.1786/8)


def move(angle):
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), error)):
        continue
    return

def compare(reference, current, error) :
    #return true if error < reference - current
    if ((abs(reference[0] - current[0][0]) > error) and (abs(reference[1] - current[1][0]) > error)):
        return True
    else:
        return False

def left90deg():
    angle = 4.4
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return

def right90deg():
    angle = -4.4
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while (compare(interface.getMotorAngleReferences(motors), interface.getMotorAngles(motors), 0.1)):
        continue
    return

def move10():
    move(3.85)
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
    print random.gauss(0, 1)
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




printSquare()

printCloud()

move40()
left90deg()
updateCloudT(math.radians(90))
printCloud()

move40()
left90deg()
updateCloudT(math.radians(90))
printCloud()

move40()
left90deg()
updateCloudT(math.radians(90))
printCloud()

move40()
left90deg()
updateCloudT(math.radians(90))
printCloud()
    
interface.terminate()
