#!/usr/bin/env bash
# Run from a staged release; deliberately does not install Klipper or state.
set -euo pipefail
stage=${1:?Pass extracted release directory}
target="$HOME/arm-control"
python="$target/venv/bin/python"
test -x "$python"
test -f "$target/features.json" || { echo 'Prepare features.json before deploying' >&2; exit 1; }
sudo -n -l /usr/bin/systemctl restart arm-control >/dev/null
"$python" - <<'PY'
import json, urllib.request
state=json.load(urllib.request.urlopen('http://127.0.0.1:8765/state', timeout=5))
if state.get('busy'):
    raise SystemExit('Controller is busy; stop the route before deployment')
PY
"$python" -m compileall -q "$stage/software/host"
for source in "$stage"/software/host/*.py; do
    name=$(basename "$source")
    if [[ "$name" == robot_mapper_klipper.py ]]; then name=robot_mapper.py; fi
    install -m 644 "$source" "$target/$name"
done
install -m 644 "$stage/software/host/requirements-camera.txt" "$target/requirements-camera.txt"
sudo -n /usr/bin/systemctl restart arm-control
for attempt in {1..20}; do
    if curl -fsS --max-time 2 http://127.0.0.1:8765/state >/dev/null; then exit 0; fi
    sleep 1
done
echo 'Host did not become healthy; inspect arm-control.service logs' >&2
exit 1
