import heapq;
import random;
import Queue;
import json;
from Function import FunctionValueGenerator;
import math;
import numpy as np;
from scipy.optimize import minimize

############################################################################
# Read from config #
############################################################################
try:
    with open("sensor_config.json", 'r') as f:
        c = json.load(f)
except IOError:
    print IOError
    print("Error reading from config file: using default configuration")
###########################################################################

class sampst:
	def __init__(self):
		self.q = Queue.Queue()
		self.counter = 0.0
		self.time = 0.0

	def add(self, time, length):
		if self.q.empty():
			tuple_ = time, length
			self.q.put(tuple_)
			return
		t = self.q.get()
		prev_time = t[0]
		prev_len = t[1]
		self.counter = self.counter + (time - prev_time) * prev_len
		self.time = time
		tuple_ = time, length
		self.q.put(tuple_)		

	def get_avg(self):
		return self.counter / self.time 


class Reading:
    def __init__(self, sensor_name, sensing_time, size, reading_value):
        self.sensor_name = sensor_name
        self.sensing_time = sensing_time
        self.size = size
	self.reading_value = reading_value

    def __repr__(self):
        return 'Sensor::%s, Time:%f, Value:%f' % (self.sensor_name, self.sensing_time, self.reading_value)

class DataAnalyzer(object):
	def __init__(self, f, q):
		self.f = f
		self.q = q

	def get_error(self):
		total = 0.0
		reading = self.q.get()
		last_known_t = reading.sensing_time
		smallest_t = last_known_t
		last_known_val = reading.reading_value

		while not self.q.empty():
			current_reading = self.q.get()
			a = self.f.integrate(current_reading.sensing_time, float(last_known_t))
			b = 0.5*(current_reading.reading_value + last_known_val)*(current_reading.sensing_time-last_known_t)
			total = total + math.fabs(a - b) * math.fabs(a - b)
			last_known_t = current_reading.sensing_time
			last_known_val = current_reading.reading_value
		if smallest_t == last_known_t :
			return 0
		return total / self.f.integrate(smallest_t, last_known_t)

class Callable:
    def call(sim):
        raise Exception('Call of %s is not implemented' % self)


class Sensor(Callable):

    def __init__(self, name, readlatency, period, size, gamma, amplitude, omega, mean, sf):
        self.name = name
        self.readlatency = readlatency
        self.period = period
        self.size = size
        self.gamma = gamma
        self.flag = False
	self.f = FunctionValueGenerator(amplitude, omega, mean)
	q = Queue.Queue()
	self.analyzer = DataAnalyzer(self.f, q)
	self.accumulated_reading = 0.0
	self.accumulated_squared_reading = 0.0
	self.total_reading_count = 0
	self.accumulated = 0.0
	self.accumulated_squared = 0.0
	self.accumulated_diff = 0.0
	self.max_in_interval = -99999.0
	self.min_in_interval = +99999.0
	self.prev_value = -1.0
	self.sf = sf

    def __repr__(self):
        return 'Sensor::%s' % self.name

    def clear(self):
	self.accumulated = 0.0
	self.accumulated_squared = 0.0
	self.total_reading_count = 0
	self.max_in_interval = -99999.0
	self.min_in_interval = +99999.0

    def set_period(self, period):
        self.period = period

    def call(self, sim):
        if self.flag:
            self.flag = False
            #print 'Time %f sensor %s reading completed' % (sim.simclock, self)
            sim.add_event(sim.simclock + self.period - self.readlatency, self)
        else:
            self.flag = True
            sim.read_queue = sim.read_queue + self.size
	    value = self.f.get_value(sim.simclock)
            reading = Reading(self.name, sim.simclock, self.size, value)
            sim.readings_queue.put(reading)
            #print 'Time %f reading sensor %s current queue %d' % (sim.simclock, self, sim.read_queue)
            sim.add_event(sim.simclock + self.readlatency, self)
	    alpha = self.sf
            self.accumulated_reading = self.accumulated_reading * (1-alpha) + value * (alpha)
	    self.accumulated_squared_reading = self.accumulated_squared_reading * (1-alpha) + value * value * alpha
	    self.accumulated = self.accumulated + value
	    self.accumulated_squared = self.accumulated_squared + value * value
	    self.total_reading_count = self.total_reading_count + 1
	    if value > self.max_in_interval:
		self.max_in_interval = value
	    if value < self.min_in_interval:
		self.min_in_interval = value
	    if self.prev_value > 0:
		self.accumulated_diff = self.accumulated_diff * (1-alpha) + math.fabs(value - self.prev_value) * alpha
	    self.prev_value = value

class Uploader(Callable):

    # here, bandwidth, up_time and down_time ... these are parameters that will vary from
    # network to network, place to place

    def __init__(self, period, bandwidth, upload_rate, up_time, down_time):
        self.period = period
        self.bandwidth = bandwidth
        self.upload_rate = upload_rate
        self.last_uploadtime = 0
        self.last_uploaded = 0

        self.up_time = up_time
        self.down_time = down_time

        self.failed = False
        self.flag = False
        self.currently_uploading = Queue.Queue()

    def __repr__(self):
        return 'Uploader'

    def call(self, sim):
        if self.failed:

            while not self.currently_uploading.empty():
                reading = self.currently_uploading.get()
                #print '-----%f delay -- failed!' % (sim.simclock - reading.sensing_time)

            return

        if self.flag:
            self.flag = False
            #print 'Time %f Upload completed' % sim.simclock

            while not self.currently_uploading.empty():
                reading = self.currently_uploading.get()
		for s in sim.sensors:
			if s.name == reading.sensor_name:
				s.analyzer.q.put(reading)
				break
                #print '----- %f delay encountered' % (sim.simclock - reading.sensing_time)
		sim.total_delay = sim.total_delay + (sim.simclock - reading.sensing_time)
		sim.total_sent = sim.total_sent + 1

            sim.add_event(sim.simclock + self.period, self)
        else:
            bytes_to_upload = max(self.upload_rate * sim.simclock - self.last_uploaded, 0)
            bytes_to_be_uploaded = 0

            while (bytes_to_be_uploaded < bytes_to_upload) and not sim.readings_queue.empty():
                reading = sim.readings_queue.get()
                self.currently_uploading.put(reading)
                bytes_to_be_uploaded = bytes_to_be_uploaded + reading.size + 5
            bytes_to_upload = bytes_to_be_uploaded

            upload_duration = 1.0 * bytes_to_upload / self.bandwidth
            sim.read_queue = max(sim.read_queue - bytes_to_upload, 0)
            self.last_uploaded = self.last_uploaded + bytes_to_upload

            #print 'Time %f UPLOADING %d bytes, remaining %d in queue' % (sim.simclock, bytes_to_upload, sim.read_queue)
	    #print 'So far uploaded %d bytes' % self.last_uploaded
            self.flag = True
            sim.add_event(sim.simclock + upload_duration, self)


class PeriodUpdater(Callable):
    def __init__(self, sensors, time_gap, uploader, choice):
        self.sensors = sensors
        self.interval = time_gap
        self.uploader = uploader
	self.choice = choice

    def __repr__(self, choice):
        return "Period updater"

    def update(self, sim, choice):
	sim.add_event(sim.simclock + self.interval, self)
	if choice == 1:
		k = len(sim.sensors)
		alpha = c["params"]["alpha"]
		beta = c["params"]["beta"]
		rate = self.uploader.upload_rate
		T = self.uploader.period
		if rate*T == alpha:
			return
		for i in range(0, k):
			pi = 1.0 * k * (sim.sensors[i].size + 5) * beta * T / (rate * T - alpha)
			sim.sensors[i].set_period(pi)
	if choice == 2:
		k = len(sim.sensors)
		total = 0.0
		w = []
		for i in range(0, k):
			e_x2 = sim.sensors[i].accumulated_squared_reading
			e_x = sim.sensors[i].accumulated_reading
			var = math.sqrt(math.fabs(e_x2 - e_x*e_x))
			cv = var / e_x
			w.append(cv)
			total = total + cv
		alpha = c["params"]["alpha"]
		beta = c["params"]["beta"]
		rate = self.uploader.upload_rate
		T = self.uploader.period
		if rate*T == alpha:
			return
		for i in range(0, k):
			wt = w[i] / total
			if wt == 0:
				return
		for i in range(0, k):
			wt = w[i] / total
			pi = 1.0 * (sim.sensors[i].size + 5) * beta * T / ( (rate * T - alpha) * wt )
			sim.sensors[i].set_period(pi)
	if choice == 3:
		k = len(sim.sensors)
		total = 0.0
		w = []
		for i in range(0, k):
			if sim.sensors[i].total_reading_count == 0:
				return
			e_x2 = sim.sensors[i].accumulated_squared / sim.sensors[i].total_reading_count
			e_x = sim.sensors[i].accumulated / sim.sensors[i].total_reading_count
			var = math.sqrt(math.fabs(e_x2 - e_x*e_x))
			cv = var / e_x
			w.append(cv)
			total = total + cv
		alpha = c["params"]["alpha"]
		beta = c["params"]["beta"]
		rate = self.uploader.upload_rate
		T = self.uploader.period
		if rate*T == alpha:
			return
		for i in range(0, k):
			wt = w[i] / total
			if wt == 0:
				return				
			pi = 1.0 * (sim.sensors[i].size + 5) * beta * T / ( (rate * T - alpha) * wt )
			sim.sensors[i].set_period(pi)
		for s in sim.sensors:
			s.clear()
	if choice == 4:
		k = len(sim.sensors)
		total = 0.0
		w = []
		for i in range(0, k):
			frac = sim.sensors[i].accumulated_diff/(sim.sensors[i].f.integrate(sim.simclock-self.interval, sim.simclock))
			w.append(frac)
			total = total + frac
		alpha = c["params"]["alpha"]
		beta = c["params"]["beta"]
		rate = self.uploader.upload_rate
		T = self.uploader.period
		if rate*T == alpha:
			return
		for i in range(0, k):
			wt = w[i] / total
			if wt == 0:
				return
			pi = 1.0 * (sim.sensors[i].size + 5) * beta * T / ( (rate * T - alpha) * wt )
			sim.sensors[i].set_period(pi)
		for s in sim.sensors:
			s.clear()
	if choice == 5:
		periods = []
		for s in sim.sensors:
			periods.append(s.period)
		x0 = np.array(periods)
		res = minimize(self.fn, x0, method='nelder-mead', options={'xtol': 1e-8, 'disp': False})
		x = res.x
		for pi in x:
			if pi < 0:
				return
		k = len(x)
		for i in range(0, k):
			sim.sensors[i].period = x[i]
	for s in sim.sensors:
		s.period = s.period * (1 + sim.deviation)
    
    def fn(self, x):
	alpha = c["params"]["alpha"]
	beta = c["params"]["beta"]
	rate = self.uploader.upload_rate
	T = self.uploader.period
	A = (rate*T - alpha)/(beta*T)			
	tot = 0.0
	for s in sim.sensors:
		tot = tot + (s.size + 5) / s.period
	d = tot - A
	return 0.5 * d * d
	

    def call(self, sim):
	self.update(sim, self.choice)

class RateUpdater(Callable):
    def __init__(self, time_gap, uploader):
        self.interval = time_gap
        self.uploader = uploader

    def __repr__(self):																																																																																																						
        return "Period updater"

    def call(self, sim):
	M = c["params"]["M"]
	t = sim.simclock
	if (M == t):
		return
	rate = self.uploader.upload_rate
	u = self.uploader.last_uploaded
	self.uploader.upload_rate = min((c["params"]["D"] - u) * 1.0 / (M - t), c["params"]["max_rate"])
	sim.add_event(sim.simclock + self.interval, self)

class FailureHandler(Callable):
    def __init__(self, uploader):
        self.uploader = uploader
        self.flag = False

    def call(self, sim):
        if self.flag:
            duration = random.expovariate(1.0 / self.uploader.up_time)
            self.uploader.failed = False
            self.flag = False
            print 'Time %f Uploader Up' % sim.simclock
            sim.add_event(sim.simclock, self.uploader)
            sim.add_event(sim.simclock + duration, self)
        else:
            duration = random.expovariate(1.0 / self.uploader.down_time)
            self.uploader.failed = True
            print 'Time %f Uploader down' % sim.simclock
            self.flag = True
            sim.add_event(sim.simclock + duration, self)


class Simulator:
    def __init__(self, seed, choice, sf, update_rate, T, deviation):
        self.simclock = 0.0
        self.event_queue = []
        self.readings_queue = Queue.Queue()
        self.read_queue = 0
	self.total_delay = 0.0
	self.total_sent = 0
	self.sf = sf
	self.choice = choice
	self.update_rate = update_rate
	self.T = T
	self.deviation = deviation
	self.s = sampst()
        random.seed(seed)

    def set_endtime(self, time):
        self.endtime = time

    def init_scene(self):
        self.sensors = []
        num_sensors = len(c["sensors"])
        for i in range(0, num_sensors):
            s1 = Sensor(c["sensors"][i]["name"], c["sensors"][i]["readlatency"], c["sensors"][i]["period"],
                    c["sensors"][i]["size"], c["sensors"][i]["gamma"], c["sensors"][i]["amplitude"], 
		    c["sensors"][i]["omega"], c["sensors"][i]["mean"], self.sf)
            self.sensors.append(s1)

        self.bought_data = c["params"]["D"]
	rate = 1.0 * self.bought_data / end_time

        upload_interval = self.T
        u = Uploader(upload_interval, 100, rate, 3600, 10)
	self.u = u
        f = FailureHandler(u)

	period_update_interval = 30
        p = PeriodUpdater(self.sensors, period_update_interval, u, self.choice)

	rate_update_interval = 200
	r = RateUpdater(rate_update_interval, u)

        for s in self.sensors:
            self.add_event(0, s)
        self.add_event(0, u)
        self.add_event(0, f)
        self.add_event(100, p)
	if self.update_rate:
		self.add_event(200, r)

	self.s.add(0, 0)

    def add_event(self, time, event):
        heapq.heappush(self.event_queue, (time, event))

    def run(self):
        while len(self.event_queue) > 0:
            time, event = heapq.heappop(self.event_queue)
            if time > self.endtime:
		self.tot = 0.0
		k = len(self.sensors)
		for i in range (0, k):
			if i == k - 1:
				self.prominent_error = self.sensors[i].analyzer.get_error()
				self.tot = self.tot + self.prominent_error
			else:
				self.tot = self.tot + self.sensors[i].analyzer.get_error()
		self.avg_delay = 1.0 * self.total_delay / self.total_sent
		self.utilization = 100.0 * self.u.last_uploaded / c["params"]["D"]
		break

            self.simclock = time
	    self.s.add(time, self.read_queue)
            event.call(self)

    def get_data(self):
	return self.tot, self.prominent_error, self.avg_delay, self.utilization

if __name__ == '__main__':

    f1 = open("total_loss vs deviation", "w")
    f2 = open("avg_queue_len deviation", "w")
    
    f1.write("deviation loss\n")
    f2.write("deviation queue_len\n")

    for deviation in range(-70, 70, 5):

	    T = 10
	    dev = 1.0 * deviation / 100.0
	    sim = Simulator(123, 1, 0.8, False, T, dev)
	    end_time = c["params"]["M"]
	    sim.set_endtime(end_time)
	    sim.init_scene()
	    sim.run()
	    t1 = sim.get_data()
	    print "stage 1 done"

	    f1.write('%d %f\n' % (deviation, t1[0]) )
	    f2.write('%d %f\n' % (deviation, sim.s.get_avg()) )

	    f1.flush()
	    f2.flush()

    f1.close()
    f2.close()
