import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams0 = interface.MotorAngleControllerParameters()
motorParams0.maxRotationAcceleration = 6.0
motorParams0.maxRotationSpeed = 12.0
motorParams0.feedForwardGain = 255/17.0
motorParams0.minPWM = 25.0
motorParams0.pidParameters.minOutput = -255
motorParams0.pidParameters.maxOutput = 255
motorParams0.pidParameters.k_p = 600.0
motorParams0.pidParameters.k_i = 500.0
motorParams0.pidParameters.k_d = 200.0

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

interface.setMotorAngleControllerParameters(motors[0],motorParams0)
interface.setMotorAngleControllerParameters(motors[1],motorParams1)


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
