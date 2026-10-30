"""GPL-3.0-only. Native builds of target runtime algorithms versus host libm.

No target execution is claimed. The DLL is linked without libc, so the probe
functions exercise Zig's supplied math implementations rather than host libm.
"""
import ctypes
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def test_zig_runtime_matches_host_libm():
    out = ROOT / "build" / "host" / "runtime"
    out.mkdir(parents=True, exist_ok=True)
    lib = out / ("math_probe.dll" if os.name == "nt" else "libmath_probe.so")
    env = dict(os.environ, ZIG_LOCAL_CACHE_DIR=str(out / "local"), ZIG_GLOBAL_CACHE_DIR=str(out / "global"))
    cmd = [sys.executable, "-m", "ziglang", "build-lib", "-dynamic", "-O", "ReleaseSmall",
           "-fcompiler-rt", "--dep", "runtime",
           f"-Mroot={ROOT/'tests/test_hal_runtime.zig'}", f"-Mruntime={ROOT/'boot/runtime.zig'}",
           f"-femit-bin={lib}"]
    result = subprocess.run(cmd, capture_output=True, text=True, env=env)
    assert result.returncode == 0, result.stdout+result.stderr
    dll = ctypes.CDLL(str(lib))
    rng = random.Random(405)
    count = 0
    errors = {}
    relative_errors = {}
    def compare(name, xs, reference, rel=3e-14, abs_tol=2e-15, float32=False):
        nonlocal count
        fn = getattr(dll, "rt_"+name)
        fn.argtypes = [ctypes.c_float if float32 else ctypes.c_double]
        fn.restype = ctypes.c_float if float32 else ctypes.c_double
        max_error = 0.0
        max_relative = 0.0
        for value in xs:
            x = ctypes.c_float(value).value if float32 else value
            want = reference(x)
            if float32:
                want = ctypes.c_float(want).value
            got = fn(x)
            count += 1
            if math.isnan(want):
                assert math.isnan(got), (name, x, got)
            else:
                assert math.isclose(got, want, rel_tol=rel, abs_tol=abs_tol), (name,x,got,want)
                if math.isfinite(want):
                    max_error = max(max_error, abs(got-want))
                    if want:
                        max_relative = max(max_relative, abs((got-want)/want))
        errors[name] = max_error
        relative_errors[name] = max_relative
    # Attitude increments through full geodetic angles, wide argument reduction.
    angles = [0.0, -0.0, 1e-14, -1e-14, math.pi/2, -math.pi, math.pi, 1e9, -1e9]
    angles += [rng.uniform(-100*math.pi, 100*math.pi) for _ in range(2000)]
    for name in ["sin", "cos"]:
        compare(name, angles, getattr(math, name))
    compare("sqrt", [0.0, 1e-300, 1e-20, 1.0]+[10**rng.uniform(-12,16) for _ in range(2000)], math.sqrt)
    compare("log", [1.0, 1e-300, 1e300]+[10**rng.uniform(-8,8) for _ in range(2000)], math.log)
    compare("exp", [-700.0,-1.0,0.0,1.0,700.0]+[rng.uniform(-30,30) for _ in range(2000)], math.exp)
    compare("acos", [-1.0,-1+1e-14,0.0,1-1e-14,1.0]+[rng.uniform(-1,1) for _ in range(2000)], math.acos)
    scales = [rng.uniform(-4e9,4e9) for _ in range(1000)] + [-0.5,0.5,-1.0,0.0]
    compare("floor", scales, math.floor, abs_tol=0)
    compare("ceil", scales, math.ceil, abs_tol=0)
    compare("logf", [10**rng.uniform(-6,7) for _ in range(2000)], math.log, rel=2e-7, abs_tol=2e-7, float32=True)
    compare("floorf", scales, math.floor, abs_tol=0, float32=True)
    dll.rt_fmod.argtypes = [ctypes.c_double,ctypes.c_double]; dll.rt_fmod.restype = ctypes.c_double
    for x in angles:
        for modulus in [2.0, math.pi, 4294967296.0]:
            assert dll.rt_fmod(x,modulus) == math.fmod(x,modulus)
            count += 1
    dll.rt_ldexp.argtypes = [ctypes.c_double,ctypes.c_int]; dll.rt_ldexp.restype = ctypes.c_double
    for _ in range(2000):
        x=rng.uniform(-100,100); exponent=rng.randint(-900,900)
        assert dll.rt_ldexp(x,exponent) == math.ldexp(x,exponent)
        count += 1
    (out/"results.txt").write_text(f"{count} comparisons passed\nmax absolute errors: {errors}\n", encoding="utf-8")
    (out/"results.json").write_text(json.dumps({"comparisons":count,"max_absolute_errors":errors,
        "max_relative_errors":relative_errors,"double_relative_tolerance":3e-14,
        "double_absolute_tolerance":2e-15,"logf_relative_and_absolute_tolerance":2e-7},indent=2)+"\n",encoding="utf-8")
    print(f"{count} runtime comparisons passed; max absolute errors: {errors}")
