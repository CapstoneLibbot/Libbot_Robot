from __future__ import annotations

import socket
import threading
from typing import Any, Dict, Optional


class TCPServer:
    """Simple TCP server that accepts requests from the .NET backend."""

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        config = settings or {}
        self.host = config.get("host", "0.0.0.0")
        self.port = int(config.get("port", 9000))
        self.buffer_size = int(config.get("buffer_size", 4096))
        self.timeout = int(config.get("timeout", 5))
        self.socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.clients: list[tuple[socket.socket, tuple[str, int]]] = []

    def start(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        self.socket.settimeout(self.timeout)
        self.running = True

        while self.running:
            try:
                client, address = self.socket.accept()
                self.clients.append((client, address))
                self.handle_client(client, address)
            except socket.timeout:
                continue
            except OSError:
                break

    def start_in_background(self) -> None:
        self.thread = threading.Thread(target=self.start, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass

    def handle_client(self, client: socket.socket, address: tuple[str, int]) -> None:
        try:
            while self.running:
                data = client.recv(self.buffer_size)
                if not data:
                    break
                message = data.decode("utf-8", errors="ignore").strip()
                if message:
                    print(f"Received from {address}: {message}")
                    client.sendall(b"ACK")
        except (socket.timeout, OSError):
            pass
        finally:
            client.close()
            self.clients = [
                (sock, addr) for sock, addr in self.clients if addr != address
            ]

    def send(self, message: str, target: Optional[tuple[str, int]] = None) -> None:
        payload = message.encode("utf-8")
        if target is None:
            for client, _ in self.clients:
                client.sendall(payload)
            return

        for client, addr in self.clients:
            if addr == target:
                client.sendall(payload)
                break
