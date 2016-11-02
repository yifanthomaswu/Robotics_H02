import curses
import brickpi

interface=brickpi.Interface()
interface.initialize()

motors = [1,3]

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


# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

speed = 10

try:
 while True:
  char = screen.getch()
  if char == ord('b'):
     interface.setMotorRotationSpeedReferences(motors,[-speed,-speed])       
     screen.addstr(0, 0, 'reverse')
  if char == ord('q'):
   break
  elif char == curses.KEY_RIGHT:
  # print doesn't work with curses, use addstr instead
     interface.setMotorRotationSpeedReferences(motors,[-speed,speed])       
     screen.addstr(0, 0, 'right')
  elif char == curses.KEY_LEFT:
     interface.setMotorRotationSpeedReferences(motors,[speed,-speed])       
     screen.addstr(0, 0, 'left ')       
  elif char == curses.KEY_UP:
     screen.addstr(0, 0, 'up   ')
     interface.setMotorRotationSpeedReferences(motors,[speed,speed])       
  elif char == curses.KEY_DOWN:
     screen.addstr(0, 0, 'down ')
     stop()
finally:
# shut down cleanly
  curses.nocbreak(); screen.keypad(0); curses.echo()
  curses.endwin()

