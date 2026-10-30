"""GPL-3.0-only. Build and run the STM32 core library's host unit tests.

    python code/firmware/stm32/tools/host_test.py              # every tests/test_*.c
    python code/firmware/stm32/tools/host_test.py link imu     # names containing these
    python code/firmware/stm32/tools/host_test.py --target-check   # also cross-compile core/ for Cortex-M4F

Compiler: $CC if set (e.g. "gcc"), else Zig's clang from the project venv
("python -m ziglang cc"). Each tests/test_X.c is linked with the core sources
named on a "CS_SOURCES:" comment line in it (a bare name is in core/, a name
with a slash is relative to code/firmware/stm32/), or with every core/*.c if
there is none, and run. Exit status is non-zero if anything fails to build or pass.
"""
import argparse
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]          # code/firmware/stm32
CORE = ROOT / 'core'
TESTS = ROOT / 'tests'
BUILD = ROOT / 'build' / 'host'
WARNINGS = ['-Wall', '-Wextra', '-Wpedantic', '-Werror', '-Wshadow', '-Wdouble-promotion',
            '-Wno-unused-function']
TARGET = ['-target', 'thumb-freestanding-eabihf', '-mcpu=cortex_m4', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']


def compiler():
    cc = os.environ.get('CC')
    if cc:
        return shlex.split(cc)
    return [sys.executable, '-m', 'ziglang', 'cc']


def sources_for(test):
    text = test.read_text(encoding='utf-8')
    names = []
    for m in re.finditer(r'CS_SOURCES:[ \t]*([^\n*]*)', text):
        names += m.group(1).split()
    if not names:
        return sorted(CORE.glob('*.c'))
    # A bare name is in core/; a name with a slash is relative to code/firmware/stm32/.
    return [(ROOT / n) if '/' in n else (CORE / n) for n in names]


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def build_and_run(test, cc, extra):
    exe = BUILD / (test.stem + ('.exe' if os.name == 'nt' else ''))
    sources = sources_for(test)
    includes = [CORE, TESTS, ROOT / 'board', ROOT / 'app'] + sorted({s.parent for s in sources})
    cmd = cc + ['-std=c99', '-O1', '-g'] + WARNINGS + extra + [f'-I{d}' for d in includes] + [str(test)]
    cmd += [str(s) for s in sources] + ['-o', str(exe), '-lm']
    t0 = time.perf_counter()
    b = run(cmd)
    if b.returncode != 0:
        return False, f'BUILD FAILED {test.name}\n{b.stdout}{b.stderr}'
    try:
        r = run([str(exe)], cwd=str(TESTS), timeout=600)
    except (OSError, subprocess.TimeoutExpired) as error:
        # Report execution-policy/tooling failures without abandoning independent
        # tests. A blocked executable is not a passing test or a proof defect.
        return False, f'EXECUTION FAILED {test.name}\n{error}'
    dt = time.perf_counter() - t0
    out = (r.stdout + r.stderr).rstrip()
    ok = r.returncode == 0
    return ok, f'{"PASS" if ok else "FAIL"} {test.name} ({dt:.1f} s)\n{out}'


def target_check(cc):
    """Compile every core source for the flight target with the same warnings."""
    out = BUILD / 'target_check'
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    for src in sorted(CORE.glob('*.c')):
        cmd = cc + ['-std=c99', '-Os', '-ffreestanding', '-fno-common'] + WARNINGS + TARGET
        cmd += [f'-I{CORE}', '-c', str(src), '-o', str(out / (src.stem + '.o'))]
        r = run(cmd)
        if r.returncode != 0:
            ok = False
            print(f'TARGET BUILD FAILED {src.name}\n{r.stdout}{r.stderr}')
    print(f'target check: {"ok" if ok else "FAILED"}')
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('names', nargs='*', help='run only tests whose file name contains one of these')
    ap.add_argument('--target-check', action='store_true', help='also cross-compile core/*.c for Cortex-M4F')
    ap.add_argument('--cflags', default='', help='extra compiler flags')
    args = ap.parse_args()
    BUILD.mkdir(parents=True, exist_ok=True)
    cc = compiler()
    tests = sorted(TESTS.glob('test_*.c'))
    if args.names:
        tests = [t for t in tests if any(n in t.name for n in args.names)]
    ok = True
    for t in tests:
        good, report = build_and_run(t, cc, shlex.split(args.cflags))
        ok &= good
        print(report, flush=True)
    print(f'{len(tests)} test programs, {"all passed" if ok else "FAILURES"}')
    if args.target_check:
        ok &= target_check(cc)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
