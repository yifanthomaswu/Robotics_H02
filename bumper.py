import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [1,3]
speed = 7

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 48.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 100.0
motorParams.pidParameters.k_i = 000.0
motorParams.pidParameters.k_d = 000.0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)


def move(angle):
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
       continue 
    return


def left90deg():
    angle = 5.85
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
	continue
    return

def right90deg():
    angle = -5.85
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
	continue
    return

def reverse():
	move(-5.4)
	return


def isTouchedL():
	result = interface.getSensorValue(touch_portL)
	if result :
		touched = result[0]
		return touched
	else :
		print("Error reading sensor")
		return -1
def isTouchedR():
	result = interface.getSensorValue(touch_portR)
	if result :
		touched = result[0]
		return touched
	else :
		print("Error reading sensor")
		return -1

def stop():
	interface.setMotorRotationSpeedReferences(motors,[0.1,0.1])
	interface.setMotorPwm(motors[0],0)
        interface.setMotorPwm(motors[1],0)
	return


def bump():
	interface.setMotorRotationSpeedReferences(motors,[speed,speed])
	while (True):
		if isTouchedL() :
		  stop()  
		  reverse()
		  right90deg()
		  interface.setMotorRotationSpeedReferences(motors,[speed,speed])
		elif isTouchedR() :
		  stop()
		  reverse()
		  left90deg()
		  interface.setMotorRotationSpeedReferences(motors,[speed,speed])
	return

touch_portL = 0
touch_portR = 1
interface.sensorEnable(touch_portL, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touch_portR, brickpi.SensorType.SENSOR_TOUCH)

bump()
interface.terminate()
