import math


#  will imitate y = mean + amplitute * sin (omega * t + delta)
class FunctionValueGenerator(object):
	def __init__(self, amplitude, omega, mean):
		self.mean = mean
		self.omega = omega
		self.amplitude = amplitude

	def get_value(self, t):
		angle_rad = self.omega * t * 1.0 * math.pi / 180.0;
		return 1.0 * self.mean + 1.0 * self.amplitude * math.sin(angle_rad)

	def integrate(self, lower_limit, upper_limit):
		angle_lower = self.omega * lower_limit * math.pi / 180.0;
		angle_upper = self.omega * upper_limit * math.pi / 180.0;
		return self.mean * (upper_limit - lower_limit) + 180.0 / math.pi * self.amplitude * 1.0 / self.omega * (math.cos(angle_lower) - math.cos(angle_upper))
