#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist

# motor_EN_A: Pin 7
# motor_A: Pin 8, Pin 10
# motor_B: Pin 12, Pin 13
# motor_EN_B: Pin 11

class AdeeptTankRos():
	
	def __init__(self):
		self._lin_vel = 0
		self._ang_vel = 0
		self._cmd_lin_vel = 0
		self._cmd_ang_vel = 0
		self._wheel_vel = (0,0)
		self._axle_length = 0.105
		self._wheel_radius = 0.047
		
		self._Motor_A_EN = 7
		self._Motor_B_EN = 11

		self._Motor_A_Pin1 = 8
		self._Motor_A_Pin2 = 10
		self._Motor_B_Pin1 = 13
		self._Motor_B_Pin2 = 12
		
		self._PWM_A = None
		self._PWM_B = None

		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup([self._Motor_A_EN, self._Motor_B_EN, self._Motor_A_Pin1, self._Motor_A_Pin2, self._Motor_B_Pin1, self._Motor_B_Pin2], GPIO.OUT)		
		
		try:
			self._PWM_A = GPIO.PWM(self._Motor_A_EN,1000)
			self._PWM_B = GPIO.PWM(self._Motor_B_EN,1000)
		except:
			pass

		self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback, queue_size=1)
	
	def _set_motor_pins(self, a1, a2, b1, b2):
		GPIO.output([self._Motor_A_Pin1, self._Motor_A_Pin2, self._Motor_B_Pin1, self._Motor_B_Pin2], (a1, a2, b1, b2))

	def _twist_callback(self, msg):
		self._cmd_lin_vel = msg.linear.x
		self._cmd_ang_vel = msg.angular.z
		rv = self._cmd_lin_vel + (self._cmd_ang_vel * self._axle_length * 0.5)
		lv = self._cmd_lin_vel - (self._cmd_ang_vel * self._axle_length * 0.5)
		
		print "x: " + str(self._cmd_lin_vel)
		print "yaw: " + str(self._cmd_lin_vel)

		# Disable both motors
		if self._cmd_lin_vel == 0 and self._cmd_ang_vel == 0:
			GPIO.output(self._Motor_A_EN, GPIO.LOW)
			GPIO.output(self._Motor_B_EN, GPIO.LOW)
		else:
			GPIO.output(self._Motor_A_EN, GPIO.HIGH)
			GPIO.output(self._Motor_B_EN, GPIO.HIGH)

		if self._cmd_lin_vel == 0:
			# Stop
			if self._cmd_ang_vel == 0:
				self._set_motor_pins(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)
			# CCW Rotation
			elif self._cmd_ang_vel > 0:
				self._set_motor_pins(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
			# CW Rotation
			elif self._cmd_ang_vel < 0:
				self._set_motor_pins(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
		# Forward driving
		elif self._cmd_lin_vel > 0:
			# Left
			if self._cmd_ang_vel > 0:
				self._set_motor_pins(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH)
			# Right
			elif self._cmd_ang_vel < 0:
				self._set_motor_pins(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW)
			# Straight
			else:
				self._set_motor_pins(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)
		# Reverse driving
		elif self._cmd_lin_vel < 0:
			# Left
			if self._cmd_ang_vel > 0:
				self._set_motor_pins(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
			# Right
			elif self._cmd_ang_vel < 0:
				self._set_motor_pins(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
			# Straight
			else:
				self._set_motor_pins(GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW)

		self._PWM_A.start(100)
		self._PWM_B.start(100)
		self._PWM_A.ChangeDutyCycle(self._wheel_vel[0])
		self._PWM_B.ChangeDutyCycle(self._wheel_vel[1])

	def run(self):
		rospy.init_node('adeept_tank_driver')
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Set wheel velocity here
			rate.sleep()
		GPIO.cleanup()

if __name__ == '__main__':
    try:
        tank = AdeeptTankRos()
        tank.run()
    except rospy.ROSInterruptException as e:
        sys.exit('Connection Error: {}'.format(e))
