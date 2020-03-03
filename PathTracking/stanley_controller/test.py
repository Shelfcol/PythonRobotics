import numpy as np
import math

def normalize_angle(angle):

  
	while angle > np.pi:
		angle -= 2.0 * np.pi

	while angle < -np.pi:
		angle += 2.0 * np.pi

	return angle

def normalize(angle):
	# Normalize an angle to [-pi, pi].
	return (angle+math.pi)%(2*math.pi)-math.pi

for i in range(-10,10):
	print(normalize_angle(float(i)),normalize(float(i)))