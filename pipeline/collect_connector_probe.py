#!/usr/bin/env python3
"""Collect completed, score-disabled connector diagnostics without evaluating code.

Visual review is deliberately separate. A result never counts as task validation.
"""
import argparse
import ast
import json
import math
import re
from pathlib import Path


def literal(text):
    text = re.sub(r"np\.float64\(([^()]*)\)", r"\1", text)
    text = re.sub(r"(?<!\w)[+-]?(?:nan|inf)\b", "None", text)
    return ast.literal_eval(text)


def collect(metadata):
    meta = json.loads(metadata.read_text())
    wrapper = Path(meta['runner_log']).read_text()
    if 'run_trial: video ' not in wrapper or 'run status: completed' not in wrapper:
        raise ValueError('Trial/render is not complete; preserve its live process')
    if not re.search(r'task_score: 0(?:\.0)?\s', wrapper):
        raise ValueError('Expected a score-disabled diagnostic')
    folder = Path(wrapper.split(' -> ', 1)[1].splitlines()[0])
    data = {name: [] for name in ('PROBE_STATE', 'POGO_CONTACT_DETAILS', 'FLEX_STATE')}
    for line in (folder / 'run.log').read_text(errors='replace').splitlines():
        for name in data:
            if line.startswith(name + ' '):
                step, record = line[len(name)+1:].split(' ', 1)
                data[name].append({'step': int(step), 'state': literal(record)})
    if not data['PROBE_STATE'] or not data['FLEX_STATE']:
        raise ValueError('Missing probe or flex observations')
    frames = re.search(r'run_trial: (\d+) steps recorded', wrapper)
    result = dict(folder=str(folder), source=meta['source'],
                  source_sha256=meta['source_sha256'],
                  frames=int(frames.group(1)) if frames else None,
                  score=0, counts_as_validation=False, sheet_reviewed=False,
                  **data)
    result['summary'] = data['PROBE_STATE'][-1]['state']['contact_sampling']
    result['flex_aborted'] = any(x['state']['aborted'] for x in data['FLEX_STATE'])
    result['maximum_imposed_wire_force_n'] = max(
        math.sqrt(sum(v*v for v in x['state']['force_N']))
        for x in data['FLEX_STATE'])
    result['wire_loading_scope'] = (
        'stationary probe; no imposed wire flex'
        if result['maximum_imposed_wire_force_n'] == 0
        else 'imposed wire loading; inspect actual displacement separately')
    ys, normals, plastic, snapshots = [], [], 0, 0
    for record in data['POGO_CONTACT_DETAILS']:
        if not 40 <= record['step'] <= 100:
            continue
        snapshots += 1
        for contact in record['state']:
            for point, force, normal in zip(contact['points_terminal'],
                                           contact['normal_force_n'],
                                           contact['normal_world']):
                if force is None or force <= 1e-6:
                    continue
                if '/housing/' in contact['body']:
                    plastic += 1
                else:
                    ys.append(point[1]*1000)
                    normals.append(abs(normal[1]))
    result['contact_location_audit'] = dict(
        control_samples_in_observation_window=snapshots, loaded_metal_points=len(ys),
        terminal_local_contact_y_mm=[min(ys), max(ys)] if ys else None,
        minimum_abs_world_normal_y=min(normals) if normals else None,
        loaded_plastic_points=plastic,
        scope='Sparse contact locations every 5 control steps; force samples at every physics step.')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metadata', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit('Refusing to overwrite an existing reviewed result')
    result = collect(args.metadata)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'summary': result['summary'],
                      'contact_location_audit': result['contact_location_audit'],
                      'wire_loading_scope': result['wire_loading_scope'],
                      'sheet_reviewed': False}))
