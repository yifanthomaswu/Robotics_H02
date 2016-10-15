import curses
import brickpi

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
speed = 6.0

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 100.0
motorParams.pidParameters.k_i = 0.0
motorParams.pidParameters.k_d = 0.0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)



# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

try:
 while True:
  char = screen.getch()
  if char == ord('q'):
   break
  elif char == curses.KEY_RIGHT:
  # print doesn't work with curses, use addstr instead
    screen.addstr(0, 0, 'right')
  elif char == curses.KEY_LEFT:
     screen.addstr(0, 0, 'left ')       
  elif char == curses.KEY_UP:
     screen.addstr(0, 0, 'up   ')
     #interface.setMotorRotationSpeedReferences(motors,[speed,speed])       
  elif char == curses.KEY_DOWN:
     screen.addstr(0, 0, 'down ')
finally:
# shut down cleanly
  curses.nocbreak(); screen.keypad(0); curses.echo()
  curses.endwin()

