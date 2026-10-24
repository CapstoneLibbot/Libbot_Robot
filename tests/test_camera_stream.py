import threading
import unittest
import urllib.error
import urllib.request

from robot_app.hardware.imx500_scanner_server import CameraStreamServer


class _FakeCamera:
    def probe(self):
        return {"status": "ok", "device": "imx500", "camera_count": 1}

    def wait_for_frame(self, after_sequence=-1, timeout=5.0):
        return after_sequence + 1, object(), b"\xff\xd8\xff\xd9"


class CameraStreamTests(unittest.TestCase):
    def test_snapshot_requires_configured_token(self):
        server = CameraStreamServer(("127.0.0.1", 0), _FakeCamera(), "secret")
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_address[1]}/snapshot.jpg"
        try:
            with self.assertRaises(urllib.error.HTTPError) as context:
                urllib.request.urlopen(url, timeout=2)
            self.assertEqual(401, context.exception.code)

            request = urllib.request.Request(url, headers={"Authorization": "Bearer secret"})
            with urllib.request.urlopen(request, timeout=2) as response:
                self.assertEqual("image/jpeg", response.headers.get_content_type())
                self.assertEqual(b"\xff\xd8\xff\xd9", response.read())
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
