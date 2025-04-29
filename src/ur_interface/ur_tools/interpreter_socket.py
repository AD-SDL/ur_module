"""Interpreter Socket to create remote communication with Interpreter URP generator that runs on UR polyscope"""

import logging
import re
import socket
from time import sleep

UR_INTERPRETER_SOCKET = 30020


class InterpreterSocket:
    """This socket creates a remote interface with the interpreter urp program generator to create urp programs on the polyscope remotely"""

    log = logging.getLogger("interpreter.InterpreterHelper")
    STATE_REPLY_PATTERN = re.compile(r"(\w+):\W+(\d+)?")

    def __init__(
        self,
        hostname: str = None,
        port: int = UR_INTERPRETER_SOCKET,
        timeout: float = 2.0,
    ) -> None:
        """Constructor for the InterpreterSocket class.
        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.hostname = hostname
        self.port = port
        self.timeout = timeout
        self.response = None

    def connect(self) -> None:
        """
        Connects to a tool at the given address.
        """
        try:
            self.socket.connect((self.hostname, self.port))
            self.socket.settimeout(self.timeout)
        except socket.error as exc:
            self.log.error(f"Interpreter Socket Error = {exc}")
            raise exc

    def disconnect(self) -> None:
        """Closes the connection with the Interpreter socket."""
        self.socket.close()

    def get_reply(self):
        """
        read one line from the socket
        :return: text until new line
        """
        collected = b""
        while True:
            part = self.socket.recv(1)
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        self.response = collected.decode("utf-8")

    def execute_command(self, command) -> int:
        """
        Send single line command to interpreter mode, and wait for reply
        :param command:
        :return: ack, or status id
        """
        self.log.debug(f"Command: '{command}'")
        if not command.endswith("\n"):
            command += "\n"

        self.socket.send(command.encode("utf-8"))
        self.get_reply()
        self.log.debug(f"Reply: '{self.response}'")
        # parse reply, raise exception if command is discarded
        reply = self.STATE_REPLY_PATTERN.match(self.response)
        if reply.group(1) == "discard":
            raise Exception(
                "Interpreter discarded message",
                self.response,
            )
        return int(reply.group(2))

    def clear(self):
        """Clear the urp programs"""
        return self.execute_command("clear_interpreter()")

    def skip(self):
        """Skips the buffer"""
        return self.execute_command("skipbuffer")

    def abort_move(self):
        """Abort"""
        return self.execute_command("abort")

    def get_last_interpreted_id(self):
        """Gets the last interpreted ID"""
        return self.execute_command("statelastinterpreted")

    def get_last_executed_id(self):
        """Gets the last executed ID"""
        return self.execute_command("statelastexecuted")

    def get_last_cleared_id(self):
        """Get the last cleared ID"""
        return self.execute_command("statelastcleared")

    def get_unexecuted_count(self):
        """Get unexecuted line count"""
        return self.execute_command("stateunexecuted")

    def end_interpreter(self):
        """End interpreter program"""
        return self.execute_command("end_interpreter()")


if __name__ == "__main__":
    """Tests"""
    tool = InterpreterSocket(ip="164.54.116.129")
    tool.connect()
    # tool.execute_command("rq_screw_turn(1,1,3600,100,False,9)")
    # tool.execute_command("")
    # sleep(10)
    # tool.execute_command("rq_screw_turn(1,1,3600,250,False,9)")
    sleep(5)
