import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/17.0
motorParams.minPWM = 25.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 600.0
motorParams.pidParameters.k_i = 500.0
motorParams.pidParameters.k_d = 200.0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)


def move(angle):
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
    return

def left90deg():
    angle = 5.4
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
    return

def right90deg():
    angle = -5.4
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
    return

move(19.1)
time.sleep(1)
left90deg()
time.sleep(1)
move(19.1)
time.sleep(1)
left90deg()
time.sleep(1)
move(19.1)
time.sleep(1)
left90deg()
time.sleep(1)
move(19.1)
time.sleep(1)
left90deg()
time.sleep(1)
    
interface.terminate()
