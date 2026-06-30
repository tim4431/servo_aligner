import logging
from ServerClass import Server, logger
from servodriver import Servoset
from config import SERVER, SERVO_CHANNEL_LIST

logging.basicConfig(level=logging.INFO)


class STSServer(Server):
	"""ZMQ server that plugs into the lab's expctl framework and drives the servos
	from received ``Sequence`` objects during the QUEUE phase.

	The ``Servoset`` is *injected* (not created here) so the interactive console
	(``servo_server.py``) can share a single serial connection with the server it
	turns on and off. ``Server.main_loop(cond_fn)`` polls the socket every 10 ms
	and checks ``cond_fn`` each iteration, so the console can run this loop in a
	background thread and stop it cleanly by flipping a flag.
	"""

	def __init__(self, name, port, message, servos, servo_channel_list):
		super().__init__(name, port, message)
		self.servo_channel_list = servo_channel_list
		self.servos = servos
		self.servos.set_torque(True)

	def set_angle(self):
		value_list = []
		pos_mask = [0 for i in range(len(self.servo_channel_list))]
		if len(self.servo_channel_list) != len(self.seq.allChannels):
			logger.error("Servoaligner received sequence with {} channels, but only {} channels are connected".format(len(self.seq.allChannels), len(self.servo_channel_list)))
			return
		for i in range(len(self.servo_channel_list)):
			chan = self.seq.allChannels[i]
			if chan is not None:
				pos_mask[i] = 1
				set_val = chan._TransValues[0]
				val = set_val[1]  # find the first value
				value_list.append(float(val))
				if set_val[3] != val:  # Check for non-identical values
					logger.warning('Servoalinger given multiple settings in same sequence, but only takes the first!!')
			else:
				value_list.append(0)

		logger.debug("pos_mask=%s value_list=%s", pos_mask, value_list)
		self.servos.set_angle(value_list)

	def cmd_seq(self, data):
		self.seq = data  # unpack the sequence
		numChannels = 0
		for chan in self.seq.allChannels:
			if chan != None: numChannels += 1
		logger.debug("Received sequence ({} channels): {}".format(numChannels, self.seq.name))
		reply = "Received sequence ({} channels): {}".format(numChannels, self.seq.name)
		self.send_msg(self.ReplyHeader() + reply)

	def queue(self):
		self.set_angle()  # move the stage during the QUEUE phase
		return 1

	def run(self):
		return 1

	def plotdata(self):
		return [0,], [0,]


if __name__ == '__main__':

	message = """
	===========================================
	==           Servo Aligner Server 1      ==
	==                STS3032                ==
	===========================================`
	"""
	servos = Servoset(SERVER["board_id"], SERVO_CHANNEL_LIST)
	server = STSServer(SERVER["name"], SERVER["port"], message, servos, SERVO_CHANNEL_LIST)
	try:
		server.main_loop()
	finally:
		servos.close()
