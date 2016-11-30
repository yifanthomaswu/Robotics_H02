import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motor = 2
speed = 0.5

interface.motorEnable(motor)
touch_port = 0
interface.sensorEnable(0, brickpi.SensorType.SENSOR_TOUCH)

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 0.7
motorParams.maxRotationSpeed = 1.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 13.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 000.0
motorParams.pidParameters.k_i = 0.0
motorParams.pidParameters.k_d = 0.0


def stop():
        interface.setMotorRotationSpeedReference(motor,0.1)
        interface.setMotorPwm(motor,0)
        return
    
interface.setMotorAngleControllerParameters(motor,motorParams)

interface.setMotorRotationSpeedReferences([motor],[-speed])

    
time.sleep(3)
print "change"
stop()
time.sleep(0.2)

interface.setMotorRotationSpeedReferences([motor],[speed])
print "Press Ctrl+C to exit"
while True:
    time.sleep(1)
    result = interface.getSensorValue(0)
    if result:
        touched = result[0]
    result = interface.getSensorValue(1)
    if result:
        touched = result[0]
    if touched:
        break

interface.terminate()
