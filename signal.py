# Send start signal
import zmq
import yaml
import sys


class Signaller:
    def __init__(self, address, port, sstr, retries = 5, send_timeout = 1000, recv_timeout = 1000):
        self.address = address
        self.port = port
        self.sstr = sstr
        self.retries = retries
        self.send_timeout = send_timeout
        self.recv_timeout = recv_timeout

    def create_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{self.address}:{self.port}")

    def clean_socket(self):
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.close()
        self.context.term()

    def send_and_get_response(self, retries = None, send_timeout = None, recv_timeout = None):
        if retries is None:
            retries = self.retries
        if send_timeout is None:
            send_timeout = self.send_timeout
        if recv_timeout is None:
            recv_timeout = self.recv_timeout
        self.create_socket()
        self.socket.SNDTIMEO = send_timeout
        self.socket.RCVTIMEO = recv_timeout
        try:
            self.socket.send_string(self.sstr)
            reply = self.socket.recv_string()
            return reply
        except zmq.Again:
            self.clean_socket()
            if retries > 0:
                print("retrying")
                return self.send_and_get_response(retries = retries - 1, send_timeout = send_timeout, recv_timeout = recv_timeout)
            else:
                return False


if __name__ == "__main__":
    with open("config.yaml") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
        ADDRESS = config["connection"]["host"]
        SIGNAL_PORT = config["connection"]["signal_port"]

    start_signal = Signaller(ADDRESS, SIGNAL_PORT, "start")
    stop_signal = Signaller(ADDRESS, SIGNAL_PORT, "stop")

    args = sys.argv
    if len(args) > 1:
        if int(args[1]) == 1:
            reply = start_signal.send_and_get_response()
        elif int(args[1]) == 0:
            reply = stop_signal.send_and_get_response()
        if reply is False:
            print("No ack received")
        else:
            print("Ack received")