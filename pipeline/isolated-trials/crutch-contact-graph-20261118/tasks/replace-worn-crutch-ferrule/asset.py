"""Pinned USDCraft source retained for a guard-reach diagnostic only."""
import hashlib
import json
import pathlib
import urllib.request


def source_path():
    info = json.loads(pathlib.Path(__file__).with_name('source_asset.json').read_text())
    path = pathlib.Path.home()/'.cache/usdcraft'/info['revision']/info['path']
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        data = urllib.request.urlopen(
            'https://huggingface.co/datasets/'+info['repo']+'/resolve/'+
            info['revision']+'/'+info['path'], timeout=60).read()
        if hashlib.sha256(data).hexdigest() != info['sha256']:
            raise ValueError('USDCraft asset checksum mismatch')
        path.write_bytes(data)
    if hashlib.sha256(path.read_bytes()).hexdigest() != info['sha256']:
        raise ValueError('Cached USDCraft asset checksum mismatch')
    return path

