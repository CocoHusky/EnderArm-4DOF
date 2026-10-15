"""Optional capabilities and portable paths; no controller I/O at import time."""
import json
import os
from pathlib import Path

SOURCE_DIR = Path(__file__).resolve().parent
DATA_DIR = Path(os.environ.get('ENDERARM_DATA_DIR', str(Path.home() / 'arm-control'))).expanduser()
CONFIG_FILE = Path(os.environ.get('ENDERARM_CONFIG', str(DATA_DIR / 'features.json'))).expanduser()
DEFAULTS = {'rail': True, 'camera': False, 'remote_access': False, 'diagnostics': True, 'sensors': False, 'sensor_swipe_programs': False}


def load_features(path=CONFIG_FILE):
    values = dict(DEFAULTS)
    try:
        with open(path) as stream:
            configured = json.load(stream)
    except FileNotFoundError:
        return values
    if not isinstance(configured, dict):
        raise ValueError('features.json must contain an object')
    for key, value in configured.items():
        if key not in DEFAULTS or type(value) is not bool:
            raise ValueError('Feature names must be known and values must be true or false')
        values[key] = value
    return values


FEATURES = load_features()
