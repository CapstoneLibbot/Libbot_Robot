from __future__ import annotations

import json
import socket
import threading
from typing import Any, Callable, Dict, Optional


class TCPServer:
    """Simple TCP server that accepts requests from the .NET backend."""

    def __init__(
        self,
        settings: Optional[Dict[str, Any]] = None,
        request_handler: Optional[Callable[[Any], Any]] = None,
    ):
        config = settings or {}
        self.host = config.get("host", "0.0.0.0")
        self.port = int(config.get("port", 9000))
        self.buffer_size = int(config.get("buffer_size", 4096))
        self.timeout = int(config.get("timeout", 5))
        self.request_handler = request_handler
        self.socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.client_threads: list[threading.Thread] = []
        self.clients: list[tuple[socket.socket, tuple[str, int]]] = []

    def set_request_handler(self, request_handler: Callable[[Any], Any]) -> None:
        self.request_handler = request_handler

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
                client.settimeout(self.timeout)
                self.clients.append((client, address))
                thread = threading.Thread(
                    target=self.handle_client,
                    args=(client, address),
                    daemon=True,
                )
                self.client_threads.append(thread)
                thread.start()
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
        buffer = ""
        try:
            while self.running:
                try:
                    data = client.recv(self.buffer_size)
                except socket.timeout:
                    continue
                if not data:
                    if buffer.strip():
                        self._send_response(client, self.process_message(buffer.strip()))
                    break
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    message, buffer = buffer.split("\n", 1)
                    message = message.strip()
                    if message:
                        print(f"Received from {address}: {message}")
                        self._send_response(client, self.process_message(message))
        except (socket.timeout, OSError):
            pass
        finally:
            client.close()
            self.clients = [
                (sock, addr) for sock, addr in self.clients if addr != address
            ]

    def process_message(self, message: str) -> Any:
        try:
            request = json.loads(message)
        except json.JSONDecodeError as exc:
            return {
                "status": "error",
                "message": f"Invalid JSON: {exc.msg}",
            }

        if self.request_handler is None:
            return {
                "status": "ok",
                "message": "ACK",
                "request": request,
            }

        try:
            return self.request_handler(request)
        except Exception as exc:
            return {
                "status": "error",
                "message": str(exc),
            }

    def _send_response(self, client: socket.socket, response: Any) -> None:
        if isinstance(response, (dict, list)):
            payload = json.dumps(response, ensure_ascii=False)
        else:
            payload = str(response)
        client.sendall((payload + "\n").encode("utf-8"))

    def send(self, message: str, target: Optional[tuple[str, int]] = None) -> None:
        payload = message.encode("utf-8")
        if target is None:
            for client, _ in self.clients:
                try:
                    client.sendall(payload)
                except OSError:
                    continue
            return

        for client, addr in self.clients:
            if addr == target:
                try:
                    client.sendall(payload)
                except OSError:
                    pass
                break

    def send_json(self, payload: Any, target: Optional[tuple[str, int]] = None) -> None:
        self.send(json.dumps(payload, ensure_ascii=False) + "\n", target)

    def wait_until_stopped(self) -> None:
        if self.thread is not None:
            self.thread.join()
