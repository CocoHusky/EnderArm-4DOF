"""Conservative publication checks; not a replacement for manual review."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'.git', '.deps', '__pycache__', '.venv', 'venv', 'build'}
PATTERNS = {
    'personal macOS path': rb'/Users/[A-Za-z0-9_.-]+/',
    'device-specific OpenSDA identifier': rb'SDA[0-9A-F]{8}',
    'private IPv4 address': rb'(?<![\d.])(?:192\.168\.\d{1,3}\.\d{1,3}|10\.\d{1,3}\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})(?![\d.])',
    'GitHub credential': rb'gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}',
    'private key': rb'-----BEGIN (?:OPENSSH |RSA |EC )?PRIVATE KEY-----',
}
errors = []
for path in ROOT.rglob('*'):
    if not path.is_file() or any(part in SKIP for part in path.relative_to(ROOT).parts):
        continue
    data = path.read_bytes()
    if path.suffix.lower() in {'.py', '.sh', '.md', '.html', '.json', '.yml', '.svg', '.cfg', '.txt'}:
        for label, pattern in PATTERNS.items():
            if re.search(pattern, data):
                errors.append(f'{path.relative_to(ROOT)}: {label}')
    if path.suffix.lower() in {'.jpg', '.jpeg'}:
        # JPEG markers before scan data: reject EXIF/XMP, IPTC, ICC and comments.
        offset = 2
        while offset + 4 <= len(data) and data[offset] == 0xff:
            marker = data[offset+1]
            if marker in (0xda, 0xd9):
                break
            if marker in (0xe1, 0xe2, 0xed, 0xfe):
                errors.append(f'{path.relative_to(ROOT)}: image metadata marker')
            length = int.from_bytes(data[offset+2:offset+4], 'big')
            if length < 2:
                raise SystemExit('Invalid JPEG segment length')
            offset += 2 + length
    if path.suffix.lower() == '.mp4':
        if any(tag in data for tag in (b'com.apple.quicktime', b'\xa9xyz', b'GPSLatitude', b'GPSLongitude')):
            errors.append(f'{path.relative_to(ROOT)}: location/device metadata')
if errors:
    raise SystemExit('\n'.join(errors))
print('Publication pattern and media-metadata checks passed; manual review still required.')
