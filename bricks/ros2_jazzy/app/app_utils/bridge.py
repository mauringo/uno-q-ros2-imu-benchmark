"""A small, persistent connection to the Arduino Router.

Requests and replies use MessagePack directly. Numeric IMU arrays arrive as
Python lists, so the fast stream needs no additional JSON parsing.
Failed connections are discarded; the next request reconnects automatically.
"""
import os
import socket
import threading
import time

import msgpack


class Bridge:
    SOCKET_PATH = os.getenv("APP_SOCKET", "/var/run/arduino-router.sock")
    _socket = None
    _unpacker = None
    _lock = threading.Lock()
    _next_msgid = 0

    @classmethod
    def _disconnect(cls):
        if cls._socket is not None:
            cls._socket.close()
        cls._socket = None
        cls._unpacker = None

    @classmethod
    def close(cls):
        with cls._lock:
            cls._disconnect()

    @classmethod
    def call(cls, method_name, *params, timeout=1.0):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        # Serialize requests so each response goes to its caller.
        with cls._lock:
            deadline = time.monotonic() + timeout
            try:
                if cls._socket is None:
                    cls._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    cls._socket.settimeout(timeout)
                    cls._socket.connect(cls.SOCKET_PATH)
                    cls._unpacker = msgpack.Unpacker(raw=False)
                cls._next_msgid = (cls._next_msgid + 1) % (2**32)
                msgid = cls._next_msgid
                # MessagePack RPC request: [request_type, id, method, arguments].
                request = [0, msgid, method_name, list(params)]
                cls._socket.settimeout(max(0.001, deadline - time.monotonic()))
                cls._socket.sendall(msgpack.packb(request, use_bin_type=True))
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f"RPC '{method_name}' timed out")
                    cls._socket.settimeout(remaining)
                    data = cls._socket.recv(4096)
                    if not data:
                        raise ConnectionError("Arduino Router closed the connection")
                    cls._unpacker.feed(data)
                    for message in cls._unpacker:
                        if not isinstance(message, list) or len(message) != 4:
                            continue
                        # Reply: [response_type, matching_id, error, result].
                        kind, reply_id, error, result = message
                        if kind != 1 or reply_id != msgid:
                            continue
                        if error is not None:
                            raise RuntimeError(f"RPC '{method_name}': {error}")
                        return result
            except Exception:
                cls._disconnect()
                raise
