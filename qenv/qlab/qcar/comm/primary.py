# other imports
import pickle
from socket import *
from typing import Any
from queue import Queue
from threading import Lock
# custom imports
from qenv.core import ServiceModule
# from qenv.core import handle_full_queue


class ControlSocket(ServiceModule):
    """
    A network communication class that handles socket connections to a remote device.

    This class is responsible for establishing a socket connection, sending and receiving data,
    and handling reconnections in case of connection interruptions.

    Attributes:
        host_name (str): The hostname or IP address of the remote device.
        port (int): The port number to connect to on the remote device.
        retry_count (int): Counter for the number of reconnection attempts.

    Methods:
        setup(): Establishes the socket connection with the remote device.
        terminate(): Closes the socket connection.
        reconnect(queue_lock: Lock, queue: Queue): Attempts to re-establish the connection and retry the operation.
        execute(queue_lock: Lock, control_queue: Queue): Sends data from the control_queue to the remote device and receives a response.
    """

    def __init__(self, host_name: str, port: int) -> None:
        self.host_name: str = host_name
        self.port: int = port
        self.retry_count: int = 0

    def setup(self) -> None:
        self.socket: socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect((self.host_name, self.port))
        print("Successfully connected to the remote device")

    def terminate(self) -> None:
        self.socket.close()

    def reconnect(self, queue_lock: Lock, queue: Queue) -> None:
        queue_lock.release()  # Ensure the lock is released on error
        print("Remote connection reset. Reconnecting...")
        self.retry_count += 1
        if self.retry_count <= 5:
            try:
                print(f"Attempting {self.retry_count}th reconnection ...")
                self.terminate()
                self.setup()
                print("Reconnected to the remote device")
                self.execute(queue_lock, queue)  # retry
            except Exception as e:
                print(f"Reconnection failed：{e}")
                if self.retry_count == 5:
                    print("Time out, stop reconnection")
                    self.terminate()
        else:
            print("Time out, stop reconnection")
            self.terminate()

    def execute(self, queue_lock: Lock, control_queue: Queue) -> None:
        queue_lock.acquire()
        try:
            if not control_queue.empty():
                # send data to the remote device
                data: Any = control_queue.get()
                queue_lock.release()
                self.socket.sendall(pickle.dumps(data))
                # get response form the remote device
                response: bytes = self.socket.recv(1024)
                response_data = pickle.loads(response)
                print(response_data)
            else:
                queue_lock.release()
            self.retry_count = 0
        except ConnectionResetError:
            # Handle the case where the server closes the connection unexpectedly
            self.reconnect(queue_lock, control_queue)