"""Verify manufacturer inputs against the validated task manifest."""
from pathlib import Path
import hashlib,json
MANIFEST=json.loads(Path(__file__).with_name('asset_manifest.json').read_text())

def verify(path,expected):
    path=Path(path)
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    if actual!=expected:
        raise RuntimeError(f'Asset integrity mismatch for {path.name}: expected SHA256 {expected}, got {actual}. The source differs from the validated task; do not substitute it silently.')
    return actual

def verify_source(name,path):
    expected=MANIFEST['wire_guard']['sha256'] if name=='guard' else MANIFEST['manufacturer_assets'][name]['step_sha256']
    return verify(path,expected)
