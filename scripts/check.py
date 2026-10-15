"""Hardware-free checks used locally and by GitHub Actions."""
import ast
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / 'software').rglob('*.py'):
    compile(path.read_text(), str(path), 'exec')
source = ast.parse((ROOT / 'software/host/robot_mapper_klipper.py').read_text())
html = next(ast.literal_eval(node.value) for node in source.body
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == 'HTML' for target in node.targets))
with tempfile.TemporaryDirectory() as directory:
    script = Path(directory) / 'ui.js'
    script.write_text('\n'.join(re.findall(r'<script>(.*?)</script>', html, re.S)))
    subprocess.run(['node', '--check', str(script)], check=True)
subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', 'software/tests', '-v'],
               cwd=ROOT, env={**__import__('os').environ, 'PYTHONPATH': str(ROOT / 'software/host')}, check=True)
