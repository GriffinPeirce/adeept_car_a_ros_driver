#!/usr/bin/env python

import RPi.GPIO as GPIO
import motor
import rospy
import time
import ultra

from std_msgs.msg import String, Bool, Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class AdeeptCarRos():

	def __init__(self):
		self._lin_vel = 0
		self._ang_vel = 0
		self._cmd_lin_vel = 0
		self._cmd_ang_vel = 0
		self._wheel_vel = (0,0)
		self._axle_length = 0.11
		self._wheel_radius = 0.025
		self._left_motor_dir = 1
		self._right_motor_dir = 0
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BOARD)
		motor.setup()
		motor.motorStop()

		self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback, queue_size=1)
		self._sonar_pub = rospy.Publisher('sonar', Range, queue_size = 1)

	def _publish_sonar(self):

		if self._sonar_pub.get_num_connections() == 0:
			return
		now = rospy.Time.now()
		sonar = Range()
		sonar.header.stamp =  now
		sonar.radiation_type = 0
		sonar.field_of_view = 0.2618
		sonar.min_range = 0.02
		sonar.max_range = 4.0
		sonar.range = ultra.checkdist()
		self._sonar_pub.publish(sonar)

	def _twist_callback(self, msg):
		self._cmd_lin_vel = msg.linear.x
		self._cmd_ang_vel = msg.angular.z

		print "x: " + str(self._cmd_lin_vel)
		print "yaw: " + str(self._cmd_ang_vel)

		# Disable both motors
		if self._cmd_lin_vel == 0 and self._cmd_ang_vel == 0:
			motor.motorStop()
		else:
			# Forward driving
			if self._cmd_lin_vel > 0:
				self._left_motor_dir = 1
				self._right_motor_dir = 1
			# Reverse driving
			elif self._cmd_lin_vel < 0:
				self._left_motor_dir = 0
				self._right_motor_dir = 0
			# CCW Rotation
			elif self._cmd_ang_vel < 0:
				self._left_motor_dir = 1
				self._right_motor_dir = 0
			# CW Rotation
			elif self._cmd_ang_vel > 0:
				self._left_motor_dir = 0
				self._right_motor_dir = 1
			motor.motor1(1,self._left_motor_dir,100)
			motor.motor(1,self._right_motor_dir,100)

	def run(self):
		rospy.init_node('adeept_car_driver')
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self._publish_sonar()
			rate.sleep()
		motor.destroy()

if __name__ == '__main__':
    try:
        car = AdeeptCarRos()
        car.run()
    except rospy.ROSInterruptException as e:
        sys.exit('Connection Error: {}'.format(e))
