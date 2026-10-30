"""GPL-3.0-only. Decode a CORE-B v1 NOR flash image to JSON lines.

Usage: python flashlog_dump.py IMAGE.bin [--summary]
Reads only. Valid pages are ordered by wrapping uint32 page sequence. Records
carry raw payload hex; message type is application-owned. Invalid/torn pages
and records are reported to stderr and yield exit status 1, never silently
treated as valid. Tick is extended microseconds within the page's boot epoch.
"""
import argparse
import binascii
import json
from pathlib import Path
import struct
import sys
import zlib

PAGE = 256
MAGIC = 0x314C5343

def decode_image(data):
    """Return (records, pages, errors); no I/O or mutation of the image."""
    errors, pages = [], []
    if len(data) % PAGE:
        errors.append({'address': len(data)//PAGE*PAGE, 'error': 'trailing partial page'})
    for address in range(0, len(data)-PAGE+1, PAGE):
        p = data[address:address+PAGE]
        if p == b'\xff'*PAGE:
            continue
        magic, seq, epoch, used, first, protected, reserved, crc = struct.unpack_from('<IIHHQIII', p)
        if magic != MAGIC or not 32 <= used <= PAGE or zlib.crc32(p[:28]+p[32:]) != crc:
            errors.append({'address': address, 'error': 'invalid page magic, length or CRC'})
            continue
        entry = dict(address=address, sequence=seq, epoch=epoch, first_tick_us=first,
                     protected_start=None if protected == 0xffffffff else protected, records=[])
        at = 32
        while at < used:
            if at+12 > used:
                errors.append({'address': address+at, 'error': 'truncated record header'})
                break
            kind, length, tick = struct.unpack_from('<BBQ', p, at)
            end = at+10+length
            if end+2 > used:
                errors.append({'address': address+at, 'error': 'truncated record payload'})
                break
            stored, = struct.unpack_from('<H', p, end)
            if binascii.crc_hqx(p[at:end], 0xffff) != stored:
                errors.append({'address': address+at, 'error': 'record CRC mismatch'})
            else:
                entry['records'].append(dict(address=address+at, sequence=seq, epoch=epoch,
                                             type=kind, tick_us=tick, payload_hex=p[at+10:end].hex()))
            at = end+2
        pages.append(entry)
    if pages:
        newest = pages[0]['sequence']
        for p in pages[1:]:
            delta = (p['sequence']-newest) & 0xffffffff
            if 0 < delta < 0x80000000:
                newest = p['sequence']
        pages.sort(key=lambda p: -((newest-p['sequence']) & 0xffffffff))
    records = [r for p in pages for r in p['records']]
    return records, pages, errors

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('image', type=Path)
    ap.add_argument('--summary', action='store_true')
    args = ap.parse_args()
    records, pages, errors = decode_image(args.image.read_bytes())
    if args.summary:
        print(json.dumps(dict(pages=len(pages), records=len(records), errors=len(errors))))
    else:
        for record in records:
            print(json.dumps(record))
    for error in errors:
        print(json.dumps(error), file=sys.stderr)
    return int(bool(errors))

if __name__ == '__main__':
    sys.exit(main())
