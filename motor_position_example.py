import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [1,3]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 25.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 300.0
motorParams.pidParameters.k_i = 200.0
motorParams.pidParameters.k_d = 100.0

motorParams1 = interface.MotorAngleControllerParameters()
motorParams1.maxRotationAcceleration = 6.0
motorParams1.maxRotationSpeed = 12.0
motorParams1.feedForwardGain = 255/17.0
motorParams1.minPWM = 25.0
motorParams1.pidParameters.minOutput = -255
motorParams1.pidParameters.maxOutput = 255
motorParams1.pidParameters.k_p = 600.0
motorParams1.pidParameters.k_i = 500.0
motorParams1.pidParameters.k_d = 200.0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)


while True:
    angle = float(input("Enter a angle to rotate (in radians): "))

    interface.increaseMotorAngleReferences(motors,[angle,angle])
    interface.startLogging("log.txt")
    
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        if motorAngles :
            print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
        time.sleep(0.1)
    
    interface.stopLogging()
    print "Destination reached!"
    
    
interface.terminate()
