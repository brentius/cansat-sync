"""GPL-3.0-only. Reproduce the complete host/cross-build flight-software gate.

Run with the project venv Python from any working directory. Requires existing
NumPy/pytest/ziglang 0.16; never installs dependencies, flashes devices, removes
files, commits or changes raw measurements. Results get a unique ignored build
directory. SITL captures and compiler outputs are regenerated under build/.
This verifies candidate software on the host; it is not hardware qualification.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'code/firmware/stm32'


def source_hashes():
    suffixes = {'.c', '.h', '.S', '.zig', '.ld', '.py', '.json', '.inc', '.toml', '.service', '.yaml', '.sh'}
    files = list((ROOT / 'code').rglob('*')) + list((ROOT / 'sim/nav').glob('*.py'))
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(files) if p.is_file() and 'build' not in p.relative_to(ROOT).parts
            and '__pycache__' not in p.parts and (p.suffix in suffixes or p.name == 'Makefile')}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python-only', action='store_true', help='skip firmware generation; bridge tests may skip')
    args = parser.parse_args(argv)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = FW / 'build/verification' / f'{stamp}-{uuid4().hex[:8]}'
    output.mkdir(parents=True, exist_ok=False)
    env = {**os.environ, 'PYTHONPATH': str(ROOT / 'code') + os.pathsep + os.environ.get('PYTHONPATH', ''),
           'PYTHONIOENCODING': 'utf-8'}
    results = []
    initial_sources = source_hashes()

    def run(name, command):
        log = output / f'{name}.log'
        started = time.perf_counter()
        with log.open('w', encoding='utf-8') as stream:
            try:
                process = subprocess.run([str(c) for c in command], cwd=ROOT, env=env,
                                         stdout=stream, stderr=subprocess.STDOUT, timeout=1200)
                code = process.returncode
            except (OSError, subprocess.TimeoutExpired) as error:
                stream.write(repr(error) + '\n')
                code = 1
        result = dict(name=name, returncode=code, seconds=round(time.perf_counter()-started, 3),
                      log=str(log.relative_to(ROOT)), command=[str(c) for c in command])
        results.append(result)
        print(f'{"PASS" if code == 0 else "FAIL"} {name} ({result["seconds"]:.2f} s): {log}', flush=True)
        return code == 0

    py = sys.executable
    sitl_ok = True
    if not args.python_only:
        run('c-host-and-target-objects', [py, FW / 'tools/host_test.py', '--cflags=-O2 -g0', '--target-check'])
        run('target-linked-image', [py, FW / 'build.py'])
        sitl_ok = run('sitl-build', [py, FW / 'sitl/build.py'])
        if sitl_ok:
            for name in ('flight', 'failures', 'reset', 'wrap', 'budget'):
                # &= evaluates every scenario, retaining independent failures.
                good = run('sitl-' + name, [FW / 'build/sitl/flight.exe', name, FW / 'build/sitl'])
                sitl_ok &= good
                if good:
                    (FW / f'build/sitl/{name}.txt').write_text(
                        (output / f'sitl-{name}.log').read_text(encoding='utf-8'), encoding='utf-8')
        env['CANSAT_REQUIRE_SITL'] = '1'
    selection = [ROOT / 'code/tests', ROOT / 'code/piapp/tests',
                 FW / 'tests/test_flashlog_dump.py', FW / 'tests/test_hal_runtime.py',
                 ROOT / 'sim/nav/test_flight_replay.py']
    if not sitl_ok:
        # Do not accidentally validate stale generated captures after a build or
        # scenario failure. This run already fails its firmware gates.
        selection += ['--ignore=' + str(ROOT / 'code/tests/test_firmware_bridge.py')]
    elif not args.python_only:
        env['CANSAT_SITL_PREBUILT'] = '1'
        selection.append(FW / 'sitl/test_sitl.py')
    run('python-flight-and-bridge', [py, '-m', 'pytest', *selection, '-q', '-p', 'no:cacheprovider',
                                    '--basetemp', output / 'pytest'])
    run('pi-config', [py, '-m', 'piapp', '--config', ROOT / 'code/deploy/pi/flight.toml', '--check-config'])
    final_sources = source_hashes()
    changed = [name for name in sorted(initial_sources.keys() | final_sources.keys())
               if initial_sources.get(name) != final_sources.get(name)]
    versions = {'python': sys.version}
    for package in ('numpy', 'pytest', 'ziglang'):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    report = dict(evidence='host tests, linked cross-build and synthetic SITL only',
                  hardware_qualified=False, python_only=args.python_only,
                  passed=not changed and all(r['returncode'] == 0 for r in results), results=results,
                  versions=versions, source_sha256=final_sources, sources_changed_during_run=changed)
    if changed:
        print('FAIL: source changed during verification: ' + ', '.join(changed), flush=True)
    path = output / 'results.json'
    path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(path, flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
