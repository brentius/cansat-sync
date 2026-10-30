"""GPL-3.0-only. Reproducible full STM32F405 image, no downloads or installs.

Run with the project venv Python (ziglang 0.16.0), or --gcc arm-none-eabi-gcc.
Artifacts and compiler caches stay in ignored build/. Linker assertions and
ELF inspection enforce SRAM/CCM/stack layout. This does not execute hardware.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "build" / "target"
WARN = ["-Wall", "-Wextra", "-Wpedantic", "-Werror", "-Wshadow", "-Wdouble-promotion"]
TARGET = ["-target", "thumb-freestanding-eabihf", "-mcpu=cortex_m4", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"]


def run(cmd):
    result = subprocess.run([str(x) for x in cmd], capture_output=True, text=True)
    if result.returncode:
        print(result.stdout + result.stderr, end="")
        raise RuntimeError("command failed: " + " ".join(map(str, cmd)))
    return result.stdout


def inspect_elf(path, write_bin=True):
    """Read ELF32 little-endian load/section tables with no external tools."""
    raw = Path(path).read_bytes()
    if raw[:7] != b"\x7fELF\x01\x01\x01":
        raise ValueError("expected ELF32 little endian")
    hdr = struct.unpack_from("<16sHHIIIIIHHHHHH", raw)
    _, _, machine, _, entry, phoff, shoff, _, _, phsize, phnum, shsize, shnum, shnames = hdr
    if machine != 40 or not entry & 1:
        raise ValueError("expected ARM Thumb entry")
    sections = [struct.unpack_from("<10I", raw, shoff+i*shsize) for i in range(shnum)]
    symbols = {}
    for s in sections:
        if s[1] != 2:  # SHT_SYMTAB
            continue
        names_section = sections[s[6]]
        symbol_names = raw[names_section[4]:names_section[4]+names_section[5]]
        for offset in range(s[4], s[4]+s[5], s[9]):
            name, value, _, _, _, index = struct.unpack_from("<IIIBBH", raw, offset)
            label = symbol_names[name:].split(b"\0", 1)[0].decode()
            if label and index:
                symbols[label] = value
    names = sections[shnames]
    strings = raw[names[4]:names[4]+names[5]]
    alloc = []
    for s in sections:
        name = strings[s[0]:].split(b"\0", 1)[0].decode()
        if s[2] & 2 and s[5]:
            alloc.append({"name": name, "address": s[3], "size": s[5], "type": s[1]})
    image = bytearray()
    for i in range(phnum):
        typ, offset, _, paddr, filesz, _, _, _ = struct.unpack_from("<8I", raw, phoff+i*phsize)
        if typ != 1 or filesz == 0:
            continue
        if not 0x08000000 <= paddr < 0x08100000 or paddr+filesz > 0x08100000:
            raise ValueError(f"unexpected file-backed load segment {paddr:x}")
        end = paddr-0x08000000+filesz
        image.extend(b"\xff" * max(0, end-len(image)))
        image[paddr-0x08000000:end] = raw[offset:offset+filesz]
    sp, reset = struct.unpack_from("<II", image)
    if sp != 0x10010000 or reset != entry or sp & 7:
        raise ValueError("invalid initial vector/stack")
    if not 0x08000000 <= entry < 0x08100000:
        raise ValueError("reset handler outside flash")
    vector_section = next(s for s in alloc if s["name"] == ".isr_vector")
    if vector_section["address"] != 0x08000000 or vector_section["size"] != 98*4:
        raise ValueError("expected 16 Cortex-M plus 82 STM32F405 vectors at flash base")
    expected_vectors = {1:"Reset_Handler",15:"SysTick_Handler",28:"DMA1_Stream1_IRQHandler",
        44:"TIM2_IRQHandler",47:"I2C1_EV_IRQHandler",48:"I2C1_ER_IRQHandler",
        49:"I2C2_EV_IRQHandler",50:"I2C2_ER_IRQHandler",53:"USART1_IRQHandler",
        55:"USART3_IRQHandler",74:"DMA2_Stream2_IRQHandler"}
    vector_words = struct.unpack_from("<98I",image)
    for index, value in enumerate(vector_words[1:],1):
        if index in [7,8,9,10,13]:
            if value:
                raise ValueError(f"nonzero reserved vector {index}")
        elif value != symbols[expected_vectors.get(index,"Default_Handler")] or not value & 1:
            raise ValueError(f"incorrect/non-Thumb vector {index}")
    memory = {"flash_image_bytes": len(image), "sram_bytes": 0, "ccm_objects_bytes": 0, "stack_reserved_bytes": 16384}
    for s in alloc:
        if 0x20000000 <= s["address"] < 0x20020000:
            memory["sram_bytes"] = max(memory["sram_bytes"], s["address"]+s["size"]-0x20000000)
        if 0x10000000 <= s["address"] < 0x10010000 and s["name"] != ".stack":
            memory["ccm_objects_bytes"] += s["size"]
    memory["sram_free_bytes"] = 131072-memory["sram_bytes"]
    memory["ccm_free_after_stack_bytes"] = 65536-memory["ccm_objects_bytes"]-16384
    if min(memory["sram_free_bytes"], memory["ccm_free_after_stack_bytes"]) < 0:
        raise ValueError("memory overflow")
    if write_bin:
        Path(path).with_suffix(".bin").write_bytes(image)
    return {"entry": entry, "initial_sp": sp, "memory": memory, "sections": alloc,"symbols":symbols,
            "vectors":{name:vector_words[index] for index,name in expected_vectors.items()}}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gcc", metavar="CC", help="optional GNU Arm compiler; links its libm/libc/libgcc")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    cache = ROOT / "build" / "zig-cache"
    os.environ["ZIG_LOCAL_CACHE_DIR"] = str(cache / "local")
    os.environ["ZIG_GLOBAL_CACHE_DIR"] = str(cache / "global")
    cc = [args.gcc] if args.gcc else [sys.executable, "-m", "ziglang", "cc"]
    target = ["-mthumb", "-mcpu=cortex-m4", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"] if args.gcc else TARGET
    flags = ["-std=c99", "-Os", "-ffreestanding", "-fno-common", "-ffunction-sections", "-fdata-sections",
             "-fno-stack-protector", "-fno-unwind-tables", "-fno-asynchronous-unwind-tables"] + WARN + target
    includes = [f"-I{ROOT/p}" for p in ["board", "core", "app", "hal", "drivers"]]
    sources = sorted(p for d in ["core", "app", "board/stm32", "hal", "drivers", "boot"] for p in (ROOT/d).glob("*.c"))
    inputs = sorted(p for d in ["core","app","board","hal","drivers","boot"] for p in (ROOT/d).rglob("*")
                    if p.is_file() and p.suffix in [".c",".h",".S",".zig",".ld"] and "sitl" not in p.parts)
    inputs += [Path(__file__).resolve()]
    input_hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    if not (ROOT / "app/main.c").is_file():
        raise RuntimeError("application main.c missing; no success-stub entry point is supplied")
    objects = []
    for src in sources + [ROOT / "boot/startup.S"]:
        obj = OUT / ("_".join(src.relative_to(ROOT).with_suffix("").parts) + ".o")
        run(cc + (target if src.suffix == ".S" else flags+includes) + ["-c", src, "-o", obj])
        objects.append(obj)
    if not args.gcc:
        zig = [sys.executable, "-m", "ziglang"]
        version = run(zig + ["version"]).strip()
        if version != "0.16.0":
            raise RuntimeError(f"validated runtime requires Zig 0.16.0, got {version}")
        runtime = OUT / "runtime.o"
        run(zig + ["build-obj", ROOT/"boot/runtime.zig", "-target", "thumb-freestanding-eabihf",
                   "-mcpu", "cortex_m4", "-O", "ReleaseSmall", "-fcompiler-rt", "-fno-stack-check",
                   "-fno-stack-protector", f"-femit-bin={runtime}"])
        objects.append(runtime)
        # Preserve the original notices with any redistributed runtime/image.
        spec = importlib.util.find_spec("ziglang")
        zigroot = Path(spec.origin).parent
        notices = zigroot / "LICENSE"
        if not notices.is_file():
            raise RuntimeError("Zig licence notice missing")
        upstream = [notices, zigroot/"lib/libc/musl/COPYRIGHT"]
        # Preserve original Sun/Arm and other notices in musl math sources.
        texts = [p.read_text(encoding="utf-8") for p in upstream]
        import re
        for source in sorted((zigroot/"lib/libc/musl/src/math").glob("*.c")):
            body = source.read_text(encoding="utf-8")
            for block in re.findall(r"/\*.*?\*/", body, flags=re.S):
                if "copyright" in block.lower() and block not in texts:
                    texts.append(f"\nFrom {source.name}:\n{block}\n")
        (OUT/"RUNTIME-NOTICES.txt").write_text("\n".join(texts), encoding="utf-8")
    elf = OUT / "cansat.elf"
    linkflags = ["-nostartfiles"] if args.gcc else ["-nostdlib"]
    linkflags += [f"-Wl,-T,{ROOT/'boot/stm32f405.ld'}", "-Wl,--gc-sections", "-Wl,-e,Reset_Handler",
                  "-Wl,-z,stack-size=16384"]
    if args.gcc:
        linkflags.append(f"-Wl,-Map,{OUT/'cansat.map'}")
    run(cc + target + linkflags + objects + (["-lm", "-lc", "-lgcc"] if args.gcc else []) + ["-o", elf])
    info = inspect_elf(elf)
    symbols = info.pop("symbols")
    if not args.gcc:
        # Zig 0.16 rejects GNU -Map. Emit an explicit ELF symbol listing,
        # not a claimed linker cross-reference map.
        (OUT/"cansat.map").write_text("# ELF symbol addresses (Zig has no -Map support)\n"+"\n".join(
            f"{value:08x} {name}" for name,value in sorted(symbols.items(),key=lambda item:(item[1],item[0])))+"\n",encoding="utf-8")
    (OUT/"sections.txt").write_text("\n".join(
        f"{s['address']:08x} {s['size']:8d} {s['name']}" for s in info["sections"])+"\n", encoding="utf-8")
    info["compiler"] = args.gcc or "ziglang 0.16.0"
    info["sources"] = [str(p.relative_to(ROOT)) for p in sources]
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != input_hashes[str(p.relative_to(ROOT))] for p in inputs):
        raise RuntimeError("source changed during build; rerun to avoid a mixed snapshot")
    info["input_sha256"] = input_hashes
    info["elf_sha256"] = hashlib.sha256(elf.read_bytes()).hexdigest()
    info["bin_sha256"] = hashlib.sha256(elf.with_suffix(".bin").read_bytes()).hexdigest()
    (OUT/"image.json").write_text(json.dumps(info, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(info["memory"], indent=2))
    print(f"linked {elf}; binary {elf.with_suffix('.bin')}")


if __name__ == "__main__":
    main()
