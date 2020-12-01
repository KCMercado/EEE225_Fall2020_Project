#! /usr/bin/env python
	# This tells ROS that this code needs Python to run

# Import libraries
import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist #this imports the geometr msg from ROS's telop twist

# Set the GPIO modes
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


# Set the variables for the GPIO motor pins
	# Note the wires (input ports) for Motors C and D are flipped because
	# the L298N is facing the other direction
pinMotorA_Forwards = 12
pinMotorA_Backwards = 16
pinMotorB_Forwards = 18
pinMotorB_Backwards = 22
pinMotorC_Forwards = 32
pinMotorC_Backwards = 36
pinMotorD_Forwards = 38
pinMotorD_Backwards = 40

# Set Freq (homw many times to turn the pin on and off each second
Frequency = 20

# Set Duty Cycle (how long the pin stay ON per cycle as a percent)
	# Setting different duty cycle per wheel will allow speed control
DutyCycleA = 30
DutyCycleB = 30
DutyCycleC = 30
DutyCycleD = 30

# Set the duty cycle to 0 to stop motors
Stop = 0

# Set the GPIO Pin mode to be Output
GPIO.setup(pinMotorA_Forwards,GPIO.OUT)
GPIO.setup(pinMotorA_Backwards,GPIO.OUT)
GPIO.setup(pinMotorB_Forwards,GPIO.OUT)
GPIO.setup(pinMotorB_Backwards,GPIO.OUT)
GPIO.setup(pinMotorC_Forwards,GPIO.OUT)
GPIO.setup(pinMotorC_Backwards,GPIO.OUT)
GPIO.setup(pinMotorD_Forwards,GPIO.OUT)
GPIO.setup(pinMotorD_Backwards,GPIO.OUT)

# Set the GPIO to software PWM at 'Frequency' (Hz)
	# This will allow speed control as well
pwmMotorA_Forwards = GPIO.PWM(pinMotorA_Forwards, Frequency)
pwmMotorA_Backwards = GPIO.PWM(pinMotorA_Backwards, Frequency)
pwmMotorB_Forwards = GPIO.PWM(pinMotorB_Forwards, Frequency)
pwmMotorB_Backwards = GPIO.PWM(pinMotorB_Backwards, Frequency)
pwmMotorC_Forwards = GPIO.PWM(pinMotorC_Forwards, Frequency)
pwmMotorC_Backwards = GPIO.PWM(pinMotorC_Backwards, Frequency)
pwmMotorD_Forwards = GPIO.PWM(pinMotorD_Forwards, Frequency)
pwmMotorD_Backwards = GPIO.PWM(pinMotorD_Backwards, Frequency)

# At startup, start software PWM w/ a duty cycle of 0 (not moving)
pwmMotorA_Forwards.start(Stop)
pwmMotorA_Backwards.start(Stop)
pwmMotorB_Forwards.start(Stop)
pwmMotorB_Backwards.start(Stop)
pwmMotorC_Forwards.start(Stop)
pwmMotorC_Backwards.start(Stop)
pwmMotorD_Forwards.start(Stop)
pwmMotorD_Backwards.start(Stop)

# Turn all motors off
def stopmotors():
	pwmMotorA_Forwards.ChangeDutyCycle(Stop)
	pwmMotorA_Backwards.ChangeDutyCycle(Stop)
	pwmMotorB_Forwards.ChangeDutyCycle(Stop)
	pwmMotorB_Backwards.ChangeDutyCycle(Stop)
	pwmMotorC_Forwards.ChangeDutyCycle(Stop)
	pwmMotorC_Backwards.ChangeDutyCycle(Stop)
	pwmMotorD_Forwards.ChangeDutyCycle(Stop)
	pwmMotorD_Backwards.ChangeDutyCycle(Stop)
	
# Turn all motors to rotate forwards	
def forward():
	pwmMotorA_Forwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorA_Backwards.ChangeDutyCycle(Stop)
	pwmMotorB_Forwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorB_Backwards.ChangeDutyCycle(Stop)
	pwmMotorC_Forwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorC_Backwards.ChangeDutyCycle(Stop)
	pwmMotorD_Forwards.ChangeDutyCycle(DutyCycleD)
	pwmMotorD_Backwards.ChangeDutyCycle(Stop)

# Turn all motors to rotate backwards
def backward():
	pwmMotorA_Forwards.ChangeDutyCycle(Stop)
	pwmMotorA_Backwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorB_Forwards.ChangeDutyCycle(Stop)
	pwmMotorB_Backwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorC_Forwards.ChangeDutyCycle(Stop)
	pwmMotorC_Backwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorD_Forwards.ChangeDutyCycle(Stop)
	pwmMotorD_Backwards.ChangeDutyCycle(DutyCycleD)

# Robot car shifts left
	# Notice which wheel is rotating forward and which
	# are rotating backwards
def left():
	pwmMotorA_Forwards.ChangeDutyCycle(Stop)
	pwmMotorA_Backwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorB_Forwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorB_Backwards.ChangeDutyCycle(Stop)
	pwmMotorC_Forwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorC_Backwards.ChangeDutyCycle(Stop)
	pwmMotorD_Forwards.ChangeDutyCycle(Stop)
	pwmMotorD_Backwards.ChangeDutyCycle(DutyCycleD)

# Robot car shifts right
	# Notice which wheel is rotating forward and which
	# are rotating backwards
def right():
	pwmMotorA_Forwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorA_Backwards.ChangeDutyCycle(Stop)
	pwmMotorB_Forwards.ChangeDutyCycle(Stop)
	pwmMotorB_Backwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorC_Forwards.ChangeDutyCycle(Stop)
	pwmMotorC_Backwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorD_Forwards.ChangeDutyCycle(DutyCycleD)
	pwmMotorD_Backwards.ChangeDutyCycle(Stop)
	
# Robot car moves forward diagonal left
	# Notice which wheel is rotating forward and which
	# are rotating backwards
def forwarddiagleft():
	pwmMotorA_Forwards.ChangeDutyCycle(Stop)
	pwmMotorA_Backwards.ChangeDutyCycle(Stop)
	pwmMotorB_Forwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorB_Backwards.ChangeDutyCycle(Stop)
	pwmMotorC_Forwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorC_Backwards.ChangeDutyCycle(Stop)
	pwmMotorD_Forwards.ChangeDutyCycle(Stop)
	pwmMotorD_Backwards.ChangeDutyCycle(Stop)

# Robot car moves backward diagonal right
	# Notice which wheel is rotating forward and which
	# are rotating backwards
def backwarddiagright():
	pwmMotorA_Forwards.ChangeDutyCycle(Stop)
	pwmMotorA_Backwards.ChangeDutyCycle(Stop)
	pwmMotorB_Forwards.ChangeDutyCycle(Stop)
	pwmMotorB_Backwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorC_Forwards.ChangeDutyCycle(Stop)
	pwmMotorC_Backwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorD_Forwards.ChangeDutyCycle(Stop)
	pwmMotorD_Backwards.ChangeDutyCycle(Stop)

# Robot car moves forward diagonal right
	# Notice which wheel is rotating forward and which
	# are rotating backwards	
def forwarddiagright():
	pwmMotorA_Forwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorA_Backwards.ChangeDutyCycle(Stop)
	pwmMotorB_Forwards.ChangeDutyCycle(Stop)
	pwmMotorB_Backwards.ChangeDutyCycle(Stop)
	pwmMotorC_Forwards.ChangeDutyCycle(Stop)
	pwmMotorC_Backwards.ChangeDutyCycle(Stop)
	pwmMotorD_Forwards.ChangeDutyCycle(DutyCycleD)
	pwmMotorD_Backwards.ChangeDutyCycle(Stop)

# Robot car moves backwards diagonal left
	# Notice which wheel is rotating forward and which
	# are rotating backwards	
def backwarddiagleft():
	pwmMotorA_Forwards.ChangeDutyCycle(Stop)
	pwmMotorA_Backwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorB_Forwards.ChangeDutyCycle(Stop)
	pwmMotorB_Backwards.ChangeDutyCycle(Stop)
	pwmMotorC_Forwards.ChangeDutyCycle(Stop)
	pwmMotorC_Backwards.ChangeDutyCycle(Stop)
	pwmMotorD_Forwards.ChangeDutyCycle(Stop)
	pwmMotorD_Backwards.ChangeDutyCycle(DutyCycleD)

# Robot car rotates clockwise
	# Both motor A and C rotate forwards
	# while both motor B and D rotate backwards	
def cw():
	pwmMotorA_Forwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorA_Backwards.ChangeDutyCycle(Stop)
	pwmMotorB_Forwards.ChangeDutyCycle(Stop)
	pwmMotorB_Backwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorC_Forwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorC_Backwards.ChangeDutyCycle(Stop)
	pwmMotorD_Forwards.ChangeDutyCycle(Stop)
	pwmMotorD_Backwards.ChangeDutyCycle(DutyCycleD)

# Robot car rotates counter clockwise
	# Both motor B and D rotate forwards
	# while both motor A and C rotate backwards	
def ccw():
	pwmMotorA_Forwards.ChangeDutyCycle(Stop)
	pwmMotorA_Backwards.ChangeDutyCycle(DutyCycleA)
	pwmMotorB_Forwards.ChangeDutyCycle(DutyCycleB)
	pwmMotorB_Backwards.ChangeDutyCycle(Stop)
	pwmMotorC_Forwards.ChangeDutyCycle(Stop)
	pwmMotorC_Backwards.ChangeDutyCycle(DutyCycleC)
	pwmMotorD_Forwards.ChangeDutyCycle(DutyCycleD)
	pwmMotorD_Backwards.ChangeDutyCycle(Stop)

# Message handler
	# The CommandCallback function will handle Sting messages
	# It takes a look at the data contained with the message and
	# then takes action
def CommandCallback(commandMessage):
	command = commandMessage.data
	
	if command == 'forward':
		print('Moving forward')
		forward()
	elif command == 'backward':
		print('Moving backward')
		backward()
	elif command == 'left':
		print('Shifting left')
		left()
	elif command == 'right':
		print('Shifting right')
		right()
	elif command == 'fdleft':
		print('Moving forward diagonal left')
		forwarddiagleft()
	elif command == 'bdright':
		print('Moving backward diagonal right')
		backwarddiagright()
	elif command == 'fdright':
		print('Moving forward diagonal right')
		forwarddiagright()
	elif command == 'bdleft':
		print('Moving backward diagonal left')
		backwarddiagleft()
	elif command == 'cw':
		print('Moving clockwise')
		cw()
	elif command == 'ccw':
		print('Moving counter clockwise')
		ccw()
	elif command == 'stop':
		print('Stopping')
		stopmotors()
	else:
		print('Unknown command, stopping instead')
		stopmotors()

# Initialize the node and name it driver		
rospy.init_node('driver')

# Subscribe to the topic named "command" and specify that
	# message is a String.
# Then we provide the CommandCallback function to request
	# that it be called when a new messages comes in on that topic
rospy.Subscriber('command', String, CommandCallback)

# Block and wait for messages to come in
rospy.spin()

# When quiting code stop all motors and clean up GPIO
print('Shutting down: Stopping Motors')
stopmotors()
GPIO.cleanup()	#cleanup stops all signals from gpio pins
