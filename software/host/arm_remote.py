"""Keep the private Tailscale endpoint available with the arm UI service."""
import os
import subprocess
import threading
import time
import json

from features import DATA_DIR

DIRECTORY = str(DATA_DIR / 'tailscale')
SOCKET = DIRECTORY + '/tailscaled.sock'


def start_remote_access():
    if not os.path.isfile(DIRECTORY + '/tailscaled'):
        return

    def supervise():
        os.makedirs(DIRECTORY + '/state', mode=0o700, exist_ok=True)
        configured = False
        while True:
            # A daemon started by a previous UI process can still be alive.
            try:
                status = subprocess.run(
                    [DIRECTORY + '/tailscale', '--socket=' + SOCKET, 'status', '--json'],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=5)
                if status.returncode == 0:
                    state = json.loads(status.stdout)
                    if state.get('BackendState') == 'Running' and not configured:
                        # Both UI/video and live controller telemetry remain
                        # private to the operator's tailnet.
                        results = []
                        for port in (8765,):
                            results.append(subprocess.run(
                                [DIRECTORY + '/tailscale', '--socket=' + SOCKET,
                                 'serve', '--bg', '--yes', '--http=' + str(port),
                                 'http://127.0.0.1:' + str(port)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                timeout=15).returncode)
                        configured = all(code == 0 for code in results)
                    time.sleep(15)
                    continue
            except (OSError, ValueError, subprocess.TimeoutExpired):
                pass
            with open(DIRECTORY + '/daemon.log', 'ab') as log:
                process = subprocess.Popen(
                    [DIRECTORY + '/tailscaled', '--tun=userspace-networking',
                     '--socket=' + SOCKET, '--statedir=' + DIRECTORY + '/state'],
                    stdout=log, stderr=log)
                process.wait()
            time.sleep(5)

    threading.Thread(target=supervise, daemon=True, name='arm-tailscale').start()
