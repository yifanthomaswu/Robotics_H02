import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [1,3]
speed = 3

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
touch_port = 0
interface.sensorEnable(0, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(1, brickpi.SensorType.SENSOR_TOUCH)

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 25.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 100.0
motorParams.pidParameters.k_i = 0.0
motorParams.pidParameters.k_d = 0.0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

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
