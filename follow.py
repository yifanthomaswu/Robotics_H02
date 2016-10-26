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


def stop():
	interface.setMotorRotationSpeedReferences(motors,[0.1,0.1])
	interface.setMotorPwm(motors[0],0)
        interface.setMotorPwm(motors[1],0)
	return


def static_vars(**kwargs):
    def decorate(readUS):
        for k in kwargs:
            setattr(readUS, k, kwargs[k])
        return readUS
    return decorate

@static_vars(last=30)
def readUS():
	usReading = interface.getSensorValue(US_port)
    	if usReading :
		print(usReading)
		d = usReading[0]
		if (d == 255):
			print(readUS.last)
			return 30 #readUS.last
		else :
			readUS.last = d
			return d
	else :
		return -1

error_range = 2
def ultra(dist):
 	while (True):
		error = readUS() - dist
		print("error="),
		print(error)
		if (error < error_range and error > -error_range):
			speed = 10
			interface.setMotorRotationSpeedReferences(motors,[speed,speed])	

		elif (error) :
			speed = 5
			gain = error * 0.6
			if (gain > 3):
				gain = 3
			if (gain < -3):
				gain = -3
			print("speedL="),
			print(speed-gain)
			print("speedR="),
			print(speed+gain)
			interface.setMotorRotationSpeedReferences(motors,[speed+gain,speed-gain])	
	return	

US_port = 2
interface.sensorEnable(US_port, brickpi.SensorType.SENSOR_ULTRASONIC);

dist = 30
ultra(dist)
interface.terminate()
