"""GPL-3.0-only. Verify log decoder independently, including C-produced image."""
import binascii
import importlib.util
from pathlib import Path
import struct
import zlib
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('flashlog_dump', ROOT/'tools'/'flashlog_dump.py')
dump = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dump)

def page(seq=0, epoch=1, kind=0x10, tick=123456789, payload=b'\x00\xff\xb5\x62'):
    body = struct.pack('<BBQ', kind, len(payload), tick)+payload
    body += struct.pack('<H', binascii.crc_hqx(body, 0xffff))
    p = bytearray(b'\xff'*256)
    struct.pack_into('<IIHHQIII', p, 0, dump.MAGIC, seq, epoch, 32+len(body), tick, 0xffffffff, 0, 0)
    p[32:32+len(body)] = body
    struct.pack_into('<I', p, 28, zlib.crc32(p[:28]+p[32:]))
    return p

def test_fields_and_blank():
    records, pages, errors = dump.decode_image(b'\xff'*256+page())
    assert not errors and len(pages) == len(records) == 1
    assert records[0] == dict(address=288, sequence=0, epoch=1, type=0x10,
                              tick_us=123456789, payload_hex='00ffb562')
    assert pages[0]['protected_start'] is None

def test_physical_ring_and_sequence_wrap():
    records, _, errors = dump.decode_image(page(0)+page(1)+page(0xfffffffe)+page(0xffffffff))
    assert not errors
    assert [r['sequence'] for r in records] == [0xfffffffe, 0xffffffff, 0, 1]

def test_page_corruption():
    p = page(); p[44] ^= 1
    records, _, errors = dump.decode_image(p)
    assert not records and len(errors) == 1 and 'page' in errors[0]['error']

def test_record_corruption_with_valid_page_crc():
    p = page(); p[44] ^= 1
    struct.pack_into('<I', p, 28, zlib.crc32(p[:28]+p[32:]))
    records, _, errors = dump.decode_image(p)
    assert not records and len(errors) == 1 and errors[0]['error'] == 'record CRC mismatch'

def test_truncated_record():
    p = page(); p[33] = 250
    struct.pack_into('<I', p, 28, zlib.crc32(p[:28]+p[32:]))
    records, _, errors = dump.decode_image(p)
    assert not records and len(errors) == 1 and 'truncated' in errors[0]['error']

def test_partial_image():
    records, _, errors = dump.decode_image(page()+b'partial')
    assert len(records) == 1 and len(errors) == 1

def test_boot_epochs_do_not_reorder_by_tick():
    records, _, errors = dump.decode_image(page(1, 10, tick=4000000000)+page(2, 11, tick=0))
    assert not errors and [r['epoch'] for r in records] == [10, 11]

def test_c_produced_image():
    # host_test.py log produces this real C-encoded synthetic NOR image.
    path = ROOT/'build'/'host'/'core_b_flash.bin'
    if not path.exists():
        pytest.skip('Run host_test.py log to produce the C integration image')
    records, pages, errors = dump.decode_image(path.read_bytes())
    assert not errors and len(pages) == 1 and len(records) == 8
    assert [r['tick_us'] for r in records] == list(range(1000000, 1000008))
    assert [r['payload_hex'] for r in records] == [bytes([i]*10).hex() for i in range(8)]
