#!/usr/bin/python
"""Vendored expctl server base class, plus the Sequence pickle shells.

``Server`` is a trimmed copy of the lab expctl framework's ZMQ REP server base
class: it owns the socket, the command dispatch (SEQ / QUEUE / RUN /
GETPLOTDATA / PING) and the polling main loop. A concrete server —
``app/zmq_server.py``'s ``STSServer`` — subclasses it and implements
``queue`` / ``run`` / ``plotdata``.

Below it live the minimal **data shells** needed to unpickle the ``Sequence``
objects expctl clients send with the SEQ command: ``Sequence``, ``Channel``,
``Interval`` and ``id_trans``. Unpickling never runs ``__init__`` or methods —
it just restores the instance attributes the client set — so the shells carry
no behaviour beyond what the server reads. Clients pickle these classes from a
top-level module named ``sequence``, so ``sequence.py`` re-exports them under
that exact name; keep the two in sync.
"""
import sys
import time
from pickle import dumps, loads

import zmq
import coloredlogs, logging

# Create a logger object.
logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)

#=============================== Server Class ==================================#
class Server:
	# Create ZeroMQ context

	def __init__(self, name, port, message=''):
		self.message = message
		self.name = name
		self.port = port
		self.seq = None
		self.sock = None

		if self.message:
			print((self.message))
		self.context = zmq.Context()
		self.sock = self.context.socket(zmq.REP)
		try:
			self.sock.bind("tcp://*:{}".format(self.port))
			logger.info("Server {} started listening on {}".format(self.name, self.port))
		except Exception as e:
			logger.exception("Bind failed!")
			sys.exit()

	def send_msg(self, msg, payload=None):
		# Send a message (command) to the client with an optional payload
		if payload is None:
			#only send a string as a command
			self.sock.send_multipart([msg.encode(),])
		else:
			#send command and python object
			self.sock.send_multipart([msg.encode(), dumps(payload)])

	def recv_msg(self):
		# Receive message, return string and (optional) payload (python object)
		multipart = self.sock.recv_multipart()
		msg = multipart[0].decode()
		if len(multipart)>1:
			payload = loads(multipart[1])
			return msg, payload
		else:
			return msg, None

	def ReplyHeader(self):
		return time.strftime('['+self.name+': %b %d %H:%M:%S]') + " "

	def cmd_unknown(self, cmd=''):
		logger.warning("Unknown command {}!".format(cmd))
		self.send_msg(self.ReplyHeader() + "Unknown command {}!".format(cmd))

	def cmd_ping(self):
		logger.info("Got Ping'd!")
		self.send_msg(self.ReplyHeader() + "Got PING'd!")

	def cmd_plotdata(self):
		logger.debug("Get Plotdata")
		plt_data = self.plotdata()
		self.send_msg("DATA", plt_data)

	def cmd_seq(self, data):
		self.seq = data # unpack the sequence
		numChannels = 0
		for chan in self.seq.allChannels:
			if chan != None: numChannels += 1
		logger.debug("Received sequence ({} channels): {}".format(numChannels, self.seq.name))
		reply = "Received sequence ({} channels): {}".format(numChannels, self.seq.name)
		self.send_msg(self.ReplyHeader() + reply)

	def cmd_queue(self):
		if self.seq == None:
			logger.error('QUEUE failed. Sequence has not been imported!')
			#self.send_msg(self.ReplyHeader() + 'QUEUE failed. Sequence has not been imported!')
		else:
			success = self.queue()
			time_taken = '%.2f' % success
			logger.debug('Successfully ran sequence ({} seconds)'.format(time_taken))
			#DO not reply for QUEUE TODO figure out correct response scheme for QUEUE
			self.send_msg(self.ReplyHeader() + 'Successfully ran sequence ({} seconds)'.format(time_taken))

	def cmd_run(self):
		if self.seq == None:
			logger.error('Run() failed. Sequence has not been imported!')
			self.send_msg(self.ReplyHeader() + 'Run() failed. Sequence has not been imported!')
		else:
			success = self.run()
			time_taken = '%.2f' % success
			logger.debug('Successfully ran sequence ({} seconds)'.format(time_taken))
			self.send_msg(self.ReplyHeader() + 'Successfully ran sequence ({} seconds)'.format(time_taken))

	def main_loop(self, cond_fn=(lambda : True)):
		while cond_fn():
			ev = self.sock.poll(10)
			if ev != 0:
				try:
					command, data = self.recv_msg() # Receive a command
					print(command)
				except KeyboardInterrupt:
					logger.info("W: interrupt received, stopping…")
					break
				else:
					if command == "RUN":  # Run the sequence (if we've already received it)
						self.cmd_run()

					elif command == "SEQ": # Load in a sequence
						self.cmd_seq(data)

					elif command == "QUEUE": # Queue/arm for trigger
						self.cmd_queue()

					elif command == 'GETPLOTDATA': # Get plot data from server
						self.cmd_plotdata()

					elif command == 'PING':
						self.cmd_ping()

					else:
						self.cmd_unknown(command)

		# clean up
		self.sock.close()
		self.context.term()


#===================== Sequence pickle shells (see module docstring) ===========#
# pickle.loads reconstructs the client's objects onto these classes without
# calling __init__, so they only need to exist and be found under the module
# name recorded in the pickle ("sequence" -- re-exported by sequence.py).

def id_trans(x):
	"""Identity transform -- the default Channel transform_t / transform_v.

	Referenced by name from client pickles (Channel._transfunc_t/_transfunc_v),
	so it must stay importable even though nothing here calls it."""
	return x


class Interval(tuple):
	"""One ramp segment ``(t_start, V_start, t_stop, V_stop)`` -- a plain tuple
	with named accessors."""
	def start_t(self):
		return self[0]
	def start_V(self):
		return self[1]
	def end_t(self):
		return self[2]
	def end_V(self):
		return self[3]


class Channel:
	"""Data shell for one sequence channel. The server reads ``_TransValues``
	(the hardware-scale ``Interval`` list); all other attributes are restored
	from the pickle but unused here."""


class Sequence:
	"""Data shell for a pickled expctl sequence. The server reads ``name`` and
	``allChannels`` (a fixed-length list of ``Channel``/``None``); all other
	attributes are restored from the pickle but unused here."""
