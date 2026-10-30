"""GPL-3.0-only. Strict host SITL build or Cortex-M4F app object checks.

Run using project .venv Python. Build products always in ignored build/sitl.
"""
import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WARN = ['-Wall', '-Wextra', '-Wpedantic', '-Werror', '-Wshadow', '-Wdouble-promotion']
TARGET = ['-target', 'thumb-freestanding-eabihf', '-mcpu=cortex_m4',
          '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']


def build(target_check=False):
    out = ROOT / 'build' / 'sitl'
    out.mkdir(parents=True, exist_ok=True)
    cc = [sys.executable, '-m', 'ziglang', 'cc', '-std=c99', '-O2'] + WARN
    cc += [f'-I{ROOT / p}' for p in ('app', 'core', 'board', 'board/sitl', 'sitl')]
    app = [ROOT / 'app' / p for p in ('app.c', 'persist.c')]
    if target_check:
        for src in app + [ROOT / 'app' / 'main.c']:
            subprocess.run(cc + TARGET + ['-ffreestanding', '-c', str(src), '-o', str(out / (src.stem + '.o'))], check=True)
        print('Cortex-M4F app objects: passed (not a linked image)')
        return None
    sources = app + sorted((ROOT / 'core').glob('*.c'))
    sources += [ROOT / 'hal/hal_logic.c', ROOT / 'board/sitl/board.c', ROOT / 'sitl/scenario.c', ROOT / 'sitl/main.c']
    exe = out / 'flight.exe'
    subprocess.run(cc + [str(p) for p in sources] + ['-lm', '-o', str(exe)], check=True)
    return exe


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--target-check', action='store_true')
    ap.add_argument('--run', choices=['flight', 'failures', 'reset', 'wrap', 'budget'])
    args = ap.parse_args()
    binary = build(args.target_check)
    if args.run and binary:
        subprocess.run([str(binary), args.run, str(ROOT / 'build/sitl')], check=True)
