import logging
import re
import socket
from time import sleep

UR_INTERPRETER_SOCKET = 30020


class InterpreterHelper:
    log = logging.getLogger("interpreter.InterpreterHelper")
    STATE_REPLY_PATTERN = re.compile(r"(\w+):\W+(\d+)?")

    def __init__(self, ip, port=UR_INTERPRETER_SOCKET):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip = ip
        self.port = port

    def connect(self):
        try:
            self.socket.connect((self.ip, self.port))
        except socket.error as exc:
            self.log.error(f"socket error = {exc}")
            raise exc

    def get_reply(self):
        """
        read one line from the socket
        :return: text until new line
        """
        collected = b''
        while True:
            part = self.socket.recv(1)
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        return collected.decode("utf-8")

    def execute_command(self, command):
        """
        Send single line command to interpreter mode, and wait for reply
        :param command:
        :return: ack, or status id
        """
        self.log.debug(f"Command: '{command}'")
        if not command.endswith("\n"):
            command += "\n"

        self.socket.send(command.encode("utf-8"))
        raw_reply = self.get_reply()
        self.log.debug(f"Reply: '{raw_reply}'")
        # parse reply, raise exception if command is discarded
        reply = self.STATE_REPLY_PATTERN.match(raw_reply)
        if reply.group(1) == "discard":
            raise Exception("Interpreter discarded message", raw_reply)
        return int(reply.group(2))

    def clear(self):
        return self.execute_command("clear_interpreter()")

    def skip(self):
        return self.execute_command("skipbuffer")

    def abort_move(self):
        return self.execute_command("abort")

    def get_last_interpreted_id(self):
        return self.execute_command("statelastinterpreted")

    def get_last_executed_id(self):
        return self.execute_command("statelastexecuted")

    def get_last_cleared_id(self):
        return self.execute_command("statelastcleared")

    def get_unexecuted_count(self):
        return self.execute_command("stateunexecuted")

    def end_interpreter(self):
        return self.execute_command("end_interpreter()")

if __name__ == "__main__":
    tool = InterpreterHelper(ip="164.54.116.129")
    tool.connect()
    tool.execute_command("rq_screw_turn(1,1,3600,100,False,9)")
    # sleep(10)
    # tool.execute_command("rq_screw_turn(1,1,3600,250,False,9)")
    sleep(5)
