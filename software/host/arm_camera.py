"""One shared, demand-driven USB camera capture for the arm UI."""
import glob
import threading
import time
import subprocess
import sys
import struct


class ArmCamera:
    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0
        self.viewers = 0
        self.error = "Camera starting"
        self.worker = None

    def stream(self, handler):
        with self.condition:
            self.viewers += 1
            if self.worker is None:
                self.worker = threading.Thread(target=self._capture, daemon=True)
                self.worker.start()
        try:
            handler.send_response(200)
            handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            handler.send_header("Cache-Control", "no-store")
            handler.end_headers()
            sequence = -1
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.sequence != sequence, timeout=5)
                    frame, sequence = self.frame, self.sequence
                if frame is None:
                    continue
                handler.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                    + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            with self.condition:
                self.viewers -= 1

    def _capture(self):
        process = None
        try:
            process = subprocess.Popen([sys.executable, __file__, '--capture'],
                                       stdout=subprocess.PIPE)
            while True:
                with self.condition:
                    if not self.viewers:
                        return
                header = process.stdout.read(4)
                if len(header) != 4:
                    self.error = 'Camera unavailable — close and reopen the camera to retry'
                    return
                length = struct.unpack('!I', header)[0]
                if length > 4000000:
                    raise ValueError('Invalid camera frame')
                frame = process.stdout.read(length)
                if len(frame) != length:
                    return
                with self.condition:
                    self.frame = frame
                    self.sequence += 1
                    self.error = None
                    self.condition.notify_all()
        except Exception as exc:
            self.error = str(exc)
        finally:
            if process is not None:
                process.terminate()
                process.wait(timeout=5)
            with self.condition:
                self.worker = None

    def capture_process(self):
        cap = None
        try:
            import cv2
            while True:
                if cap is None:
                    devices = sorted(glob.glob('/dev/v4l/by-id/*-video-index0'))
                    cap = cv2.VideoCapture(devices[0] if devices else '/dev/video0', cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 10)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, image = cap.read()
                if not ok:
                    self.error = 'Camera disconnected — reconnect USB; video retries automatically'
                    cap.release()
                    cap = None
                    time.sleep(1)
                    continue
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    frame = jpeg.tobytes()
                    sys.stdout.buffer.write(struct.pack('!I', len(frame)) + frame)
                    sys.stdout.buffer.flush()
                time.sleep(0.1)
        except Exception as exc:
            self.error = str(exc)
        finally:
            if cap is not None:
                cap.release()
            with self.condition:
                self.worker = None


CAMERA = ArmCamera()

if __name__ == '__main__':
    CAMERA.capture_process()
