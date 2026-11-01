"""Verify publication bytes and saved-record consistency; no flight simulation."""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest = json.loads((ROOT/'PUBLICATION_MANIFEST.json').read_text())
    failures = []
    for name, expected in manifest['files'].items():
        p = ROOT/name
        if not p.is_relative_to(ROOT) or not p.is_file() or sha(p) != expected:
            failures.append(name)
    original = ROOT/'original_v1'
    for name, expected in json.loads((original/'checksums.json').read_text()).items():
        if sha(original/name) != expected:
            failures.append('original checksum: '+name)
    research = ROOT/'night_research'
    receipt = json.loads((research/'DELIVERY_RECEIPT.json').read_text())
    for name, expected in receipt['files'].items():
        if sha(research/name) != expected:
            failures.append('report receipt: '+name)
    cases = [json.loads(p.read_text()) for p in sorted((research/'results/cases').glob('*.json'))]
    if len(cases) != 620:
        failures.append('Expected 620 saved cases')
    for record in cases:
        expected = hashlib.sha256(json.dumps({'config':record['config'],
            'model_hash':record['model_sha256']},sort_keys=True).encode()).hexdigest()[:20]
        if expected != record['case_id']:
            failures.append('case id: '+record['case_id'])
        metrics = record.get('metrics')
        if metrics and metrics['config'] != record['config']:
            failures.append('metric config: '+record['case_id'])
        if record.get('trace') and not (research/record['trace']).is_file():
            failures.append('trace: '+record['case_id'])
    traces = list((research/'results/traces').glob('*.npz'))
    if len(traces) != 612:
        failures.append('Expected 612 batch traces')
    if failures:
        print(json.dumps({'passed':False,'failures':failures},ensure_ascii=False,indent=2))
        return 1
    print(json.dumps({'passed':True,'files_checked':len(manifest['files']),
        'saved_cases':len(cases),'batch_traces':len(traces),
        'scope':'File and stored-data integrity only; no simulation or flight safety conclusion.'},ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
