#! /usr/bin/env python
	# This tells ROS that this code needs Python to run

# Import libraries
import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist
	#this imports the geometr msg from ROS's telop twist

# Set the GPIO modes
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

_FREQUENCY = 20
	# set the frequency into a convetionallu named variable
	# underscore means internal use only
	# all caps means it is constant

# Create a function called "_clip"
	# this ensures that value is between min and max
def _clip(value, minimum, maximum):
 
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value

# Create a class called "Motor"
class Motor:

		# Initial the class
		# it is called whenever a new instance of the class is created
		# accepts two parameter for frwrd and bckwrd pins of motor
    def __init__(self, forward_pin, backward_pin):
        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)

        self._forward_pwm = GPIO.PWM(forward_pin, _FREQUENCY)
        self._backward_pwm = GPIO.PWM(backward_pin, _FREQUENCY)

    # Move here utilize the _clip function for the spd %
    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)

        # Positive speeds move wheels forward
        # negative speeds move wheels backward
        if speed_percent < 0:
            self._backward_pwm.start(speed)
            self._forward_pwm.start(0)
        else:
            self._forward_pwm.start(speed)
            self._backward_pwm.start(0)

# Create a class called "Driver"
class Driver:

		# Initalize the class
		# it is called whenever a new instance of the class is created
    def __init__(self):
    		
    		# Initialize the ROS node
        rospy.init_node('driver')

        # Identify parameters
        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 1)
        self._rate = rospy.get_param('~rate', 10)
        
        # Create 4 instantces of the Motor class
        # assign pins to motors.
        self._MotorA = Motor(12, 16) # Front left wheel
        self._MotorB = Motor(18, 22) # Front right wheel
        self._MotorC = Motor(32, 36) # Back left wheel
        self._MotorD = Motor(38, 40) # Back right wheel
        
     		# Initialize speed to zero
        self._MotorA_speed_percent = 0
        self._MotorB_speed_percent = 0
        self._MotorC_speed_percent = 0
        self._MotorD_speed_percent = 0

        # Setup ROS subscriber for velocity twist message
        rospy.Subscriber(
            'cmd_vel', Twist, self._velocity_received_callback)
		
		# Create the ROS subscriber callback function
    def _velocity_received_callback(self, message):

        self._last_received = rospy.get_time()

        # Extract linear and angular velocities from the sub message
        linear_x = message.linear.x
        linear_y = message.linear.y
        angular_z = message.angular.z

        # Calculate wheel speeds in m/s
        MotorA_speed = linear_x - linear_y - angular_z
        MotorB_speed = linear_x + linear_y + angular_z
        MotorC_speed = linear_x + linear_y - angular_z
        MotorD_speed = linear_x - linear_y + angular_z

        # Convert m/s into percent of maximum wheel speed
        # this gives us a duty cycle that we can apply to each motor.
        self._MotorA_speed_percent = (100 * MotorA_speed)
        self._MotorB_speed_percent = (100 * MotorB_speed)
        self._MotorC_speed_percent = (100 * MotorC_speed)
        self._MotorD_speed_percent = (100 * MotorD_speed)
        
    # Create a function called "run"
    # this is control loop of the driver
    def run(self):
        
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
        
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop
            # moving
            delay = rospy.get_time() - self._last_received
            
            if delay < self._timeout:
                self._MotorA.move(self._MotorA_speed_percent)
                self._MotorB.move(self._MotorB_speed_percent)
                self._MotorC.move(self._MotorC_speed_percent)
                self._MotorD.move(self._MotorD_speed_percent)
            else:
                self._MotorA.move(0)
                self._MotorB.move(0)
                self._MotorC.move(0)
                self._MotorD.move(0)

            rate.sleep()

# Create function called "main"
	# this will create a new Driver class instance and the loop
def main():
    driver = Driver()

    # Run driver. This will block
    driver.run()

    GPIO.cleanup()

# This will run the "main" function
if __name__ == '__main__':
    main()

