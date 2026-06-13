"""ZMQ REP server base class (expctl adapter).

Port of the legacy ``ServerClass.py``. The wire protocol is unchanged —
multipart messages of a command string plus an optional pickled payload,
commands RUN / SEQ / QUEUE / GETPLOTDATA / PING — so existing expctl clients
need no changes. Requires the ``server`` extra (pyzmq).
"""

from __future__ import annotations

import logging
import sys
import time
from pickle import dumps, loads
from typing import Sequence as Seq

import numpy as np
import zmq

from .compat import install_sequence_aliases

logger = logging.getLogger(__name__)

try:
    import coloredlogs

    coloredlogs.install(level="DEBUG", logger=logger)
except ImportError:
    pass


class Server:
    """REP socket message loop; subclasses implement queue()/run()/plotdata()."""

    def __init__(
        self,
        name: str,
        port: int,
        message: str = "",
        sequence_aliases: Seq[str] = ("sequence",),
    ):
        self.message = message
        self.name = name
        self.port = port
        self.seq = None
        self.sock = None
        self.sequence_aliases = tuple(sequence_aliases)

        if self.message:
            print(self.message)
        self.context = zmq.Context()
        self.sock = self.context.socket(zmq.REP)
        try:
            self.sock.bind("tcp://*:{}".format(self.port))
            logger.info("Server %s started listening on %s", self.name, self.port)
        except Exception:
            logger.exception("Bind failed!")
            sys.exit()

    def send_msg(self, msg, payload=None):
        # send a command string to the client with an optional payload
        if payload is None:
            self.sock.send_multipart([msg.encode()])
        else:
            self.sock.send_multipart([msg.encode(), dumps(payload)])

    def recv_msg(self):
        # receive a command string and (optional) pickled payload
        multipart = self.sock.recv_multipart()
        msg = multipart[0].decode()
        if len(multipart) > 1:
            install_sequence_aliases(self.sequence_aliases)
            payload = loads(multipart[1])
            return msg, payload
        return msg, None

    def ReplyHeader(self):
        return time.strftime("[" + self.name + ": %b %d %H:%M:%S]") + " "

    def cmd_unknown(self, cmd=""):
        logger.warning("Unknown command %s!", cmd)
        self.send_msg(self.ReplyHeader() + "Unknown command {}!".format(cmd))

    def cmd_ping(self):
        logger.info("Got Ping'd!")
        self.send_msg(self.ReplyHeader() + "Got PING'd!")

    def cmd_plotdata(self):
        logger.debug("Get Plotdata")
        plt_data = self.plotdata()
        self.send_msg("DATA", plt_data)

    def plotdata(self):
        return np.zeros(10)

    def cmd_seq(self, data):
        self.seq = data  # unpack the sequence
        numChannels = 0
        for chan in self.seq.allChannels:
            if chan is not None:
                numChannels += 1
        logger.debug("Received sequence (%s channels): %s", numChannels, self.seq.name)
        reply = "Received sequence ({} channels): {}".format(numChannels, self.seq.name)
        self.send_msg(self.ReplyHeader() + reply)

    def cmd_queue(self):
        if self.seq is None:
            logger.error("QUEUE failed. Sequence has not been imported!")
        else:
            success = self.queue()
            time_taken = "%.2f" % success
            logger.debug("Successfully ran sequence (%s seconds)", time_taken)
            self.send_msg(
                self.ReplyHeader()
                + "Successfully ran sequence ({} seconds)".format(time_taken)
            )

    def queue(self):
        return 0.0

    def cmd_run(self):
        if self.seq is None:
            logger.error("Run() failed. Sequence has not been imported!")
            self.send_msg(
                self.ReplyHeader() + "Run() failed. Sequence has not been imported!"
            )
        else:
            success = self.run()
            time_taken = "%.2f" % success
            logger.debug("Successfully ran sequence (%s seconds)", time_taken)
            self.send_msg(
                self.ReplyHeader()
                + "Successfully ran sequence ({} seconds)".format(time_taken)
            )

    def run(self):
        return 0.0

    def main_loop(self, cond_fn=(lambda: True)):
        while cond_fn():
            ev = self.sock.poll(10)
            if ev != 0:
                try:
                    command, data = self.recv_msg()
                    logger.debug("command: %s", command)
                except KeyboardInterrupt:
                    logger.info("W: interrupt received, stopping…")
                    break
                else:
                    if command == "RUN":
                        self.cmd_run()
                    elif command == "SEQ":
                        self.cmd_seq(data)
                    elif command == "QUEUE":
                        self.cmd_queue()
                    elif command == "GETPLOTDATA":
                        self.cmd_plotdata()
                    elif command == "PING":
                        self.cmd_ping()
                    else:
                        self.cmd_unknown(command)

        # clean up
        self.sock.close()
        self.context.term()
