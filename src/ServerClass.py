#!/usr/bin/python
import sys
from signal import signal, SIGINT
from sys import exit
import zmq
from pickle import dumps, loads
#from utilities.util import *
import sequence
import math
import time
import numpy as np
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

	def plotdata(self):
		return DataForPlot(self.seq)

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

	def queue(self):
		return RunServer(self.seq, autostart=0)

	def cmd_run(self):
		if self.seq == None:
			logger.error('Run() failed. Sequence has not been imported!')
			self.send_msg(self.ReplyHeader() + 'Run() failed. Sequence has not been imported!')
		else:
			success = self.run()
			time_taken = '%.2f' % success
			logger.debug('Successfully ran sequence ({} seconds)'.format(time_taken))
			self.send_msg(self.ReplyHeader() + 'Successfully ran sequence ({} seconds)'.format(time_taken))

	def run(self):
		return RunServer(self.seq)

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

def DataForPlot(seq):
	return np.zeros(10)

def RunServer(seq, autostart = 1):
	time.sleep(0.11)
	return 0.11

def handler(signal_received, frame):
	# Handle any cleanup here
	print('SIGINT or CTRL-C detected. Exiting gracefully')
	exit(0)

if __name__ == '__main__':

	message = """
	===========================================
	==       Dummy Digital Output Server     ==
	==              for PCIe 6537            ==
	=========================================== 
	"""
	server = Server("DOut1", 50001, message)
	server.main_loop()