"""Local-only, dependency-light server for the centerline playground."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import threading
from urllib.parse import urlsplit

from medial_axis import Cancelled, analyze


ROOT = Path(__file__).resolve().parent
COMPUTE_LOCK = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def send_data(self, status, payload, content_type="application/json"):
        data = payload if isinstance(payload, bytes) else json.dumps(payload, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            logging.info("Browser disconnected before receiving the response.")

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            self.send_data(200, (ROOT / "index.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/health":
            self.send_data(200, {"status": "ok"})
        else:
            self.send_data(404, {"error": "Not found."})

    def do_POST(self):
        if self.path != "/api/analyze":
            self.send_data(404, {"error": "Not found."})
            return
        origin = self.headers.get("Origin")
        expected = f"http://{self.headers.get('Host')}"
        if origin and origin != expected:
            self.send_data(403, {"error": "Cross-origin requests are not allowed."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 100_000:
                raise ValueError("Request must be between 1 and 100000 bytes.")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("Request must be a JSON object.")
            if not COMPUTE_LOCK.acquire(blocking=False):
                self.send_data(503, {"error": "Another shape is being processed. Try again shortly."})
                return
        except (ValueError, TypeError) as error:
            self.send_data(400, {"error": str(error)})
            return
        # Stream newline-delimited JSON: progress events, then a result or error event.
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        def emit(event):
            try:
                self.wfile.write(json.dumps(event, allow_nan=False).encode() + b"\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                raise Cancelled()

        try:
            report = lambda fraction, message: emit({"type": "progress", "fraction": fraction, "message": message})
            result = analyze(payload.get("points"), payload.get("resolution", 240),
                             payload.get("prune", .05), payload.get("extend", True), progress=report)
            emit({"type": "result", "data": result})
        except Cancelled:
            logging.info("Browser disconnected during computation.")
        except (ValueError, TypeError) as error:
            logging.warning("Rejected shape: %s", error)
            try:
                emit({"type": "error", "error": str(error)})
            except Cancelled:
                pass
        except Exception:
            logging.exception("Unexpected geometry processing error")
            try:
                emit({"type": "error", "error": "Unexpected geometry error. See the server log for details."})
            except Cancelled:
                pass
        finally:
            COMPUTE_LOCK.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Centerline Lab: http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Centerline Lab.")
    finally:
        server.server_close()
