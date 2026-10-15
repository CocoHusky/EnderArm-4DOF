#!/usr/bin/env python3
"""Standalone local-only FRDM-KL02Z hardware test dashboard. No robot imports."""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from sensor_transport import SensorSerial

MODES = ("manual", "acceleration", "orientation", "slider", "tap", "motion")
COLORS = ("off", "red", "green", "blue", "yellow", "cyan", "magenta", "white")
PAGE = Path(__file__).with_name("sensor_lab.html")


def app(port):
    sensor = SensorSerial(port, 115200, stale_after=0.75)
    sensor.start()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send_json(self, obj, status=200):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                body = PAGE.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/api/state":
                self.send_json(sensor.snapshot())
            else:
                self.send_error(404)

        def do_POST(self):
            if urlsplit(self.path).path != "/api/command":
                self.send_error(404)
                return
            # Local-only service; no permissive CORS.
            if self.headers.get("Origin") not in (None, "http://127.0.0.1:8766", "http://localhost:8766"):
                self.send_json({"error": "invalid origin"}, 403)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size > 512 or size <= 0:
                    raise ValueError("invalid command size")
                cmd = json.loads(self.rfile.read(size))
                if not isinstance(cmd, dict):
                    raise ValueError("invalid command")
                if cmd.get("action") == "mode" and cmd.get("value") in MODES:
                    command = "MODE " + cmd["value"].upper()
                elif cmd.get("action") == "led" and cmd.get("value") in COLORS:
                    command = "LED " + cmd["value"].upper()
                else:
                    raise ValueError("unsupported command")
                if not sensor.snapshot()["fresh"]:
                    raise RuntimeError("no fresh hardware samples; command not sent")
                sensor.send_command(command)
                self.send_json({"sent": command, "note": "Verify reported mode/LED in subsequent real frames"})
            except (ValueError, RuntimeError, OSError) as exc:
                self.send_json({"error": str(exc)}, 400)

    server = ThreadingHTTPServer(("127.0.0.1", 8766), Handler)
    print("Sensor Lab: http://127.0.0.1:8766/ | board:", port, flush=True)
    try:
        server.serve_forever()
    finally:
        sensor.stop()
        server.server_close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="OpenSDA serial port for your sensor board")
    app(parser.parse_args().port)
