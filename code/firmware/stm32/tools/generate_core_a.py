"""GPL-3.0-only. Generate CORE-A wire headers and oracle fixtures from Python references.
Run with --headers once before publishing HEADERS_A.ready; default only refreshes tests.
"""
import json
import random
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parents[1]))
from cansat import link, telemetry
TYPES = {'B':'uint8_t', 'b':'int8_t', 'H':'uint16_t', 'h':'int16_t', 'I':'uint32_t', 'i':'int32_t'}
def cname(cls):
    return {'PiStatus':'pi_status','TrigCfg':'trig_cfg'}.get(cls.__name__, cls.__name__.lower())
def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')
def headers():
    out = ["/* GPL-3.0-only. C twin of cansat/link.py; structs contain wire-coded integers,",
           " * including fields whose Python names end in _pa, _m, _rad, _s or _v.",
           " * Pack/unpack use explicit little endian; reserved padding is zero on pack.",
           " * Length-returning functions return 0 on capacity/argument failure. */",
           "#ifndef CS_LINK_H", "#define CS_LINK_H", "#include <stdbool.h>", "#include <stddef.h>", "#include <stdint.h>",
           "#define CS_LINK_MAX_PAYLOAD 249u", "#define CS_LINK_MAX_FRAME 253u", "#define CS_LINK_MAX_WIRE 255u"]
    for name,value in vars(link).items():
        if name.isupper() and not name.startswith('_') and isinstance(value,int):
            out.append(f'#define CS_{name} {value}u')
    for cls in link.MESSAGES.values():
        n=cname(cls)
        out += [f'#define CS_LINK_{n.upper()}_ID 0x{cls.ID:02x}u',f'#define CS_LINK_{n.upper()}_SIZE {cls.SIZE}u', 'typedef struct {']
        for name,code,kind in cls.LAYOUT:
            if name is None: continue
            arr = f'[{code[:-1]}]' if len(code)>1 else ''
            out.append(f'    {TYPES[code[-1]]} {name}{arr};')
        out += [f'}} cs_link_{n}_t;',f'size_t cs_link_pack_{n}(uint8_t *dst, size_t cap, const cs_link_{n}_t *msg);',f'bool cs_link_unpack_{n}(cs_link_{n}_t *msg, const uint8_t *src, size_t len);']
    out += ['/* Scaling: link floor(x/step+0.5), reserve signed minimum/unsigned maximum for NA. */',
            'int32_t cs_link_scale_signed(double value, double step, unsigned bits);',
            'uint32_t cs_link_scale_unsigned(double value, double step, unsigned bits);',
            'uint8_t cs_link_angle(double radians);',
            'size_t cs_link_payload_size(uint8_t id);',
            'size_t cs_link_frame(uint8_t *dst, size_t cap, uint8_t id, uint8_t seq, const uint8_t *payload, size_t len);',
            'typedef struct { uint8_t seq; } cs_link_encoder_t;',
            'void cs_link_encoder_init(cs_link_encoder_t *encoder);',
            'size_t cs_link_encode(cs_link_encoder_t *encoder, uint8_t *dst, size_t cap, uint8_t id, const uint8_t *payload, size_t len);',
            'typedef struct {',
            '    uint64_t bytes, frames, empty, too_long, stuffing, short_frame, crc, unknown, length, lost, repeats, ignored, faults;',
            '} cs_link_counters_t;',
            '/* short_frame corresponds to Python stats["short"]. Callback payload valid only during callback. */',
            'typedef void (*cs_link_receive_fn)(void *context, uint8_t id, uint8_t seq, const uint8_t *payload, size_t len);',
            'typedef struct {',
            '    uint8_t buf[CS_LINK_MAX_WIRE - 1u], last_seq;',
            '    size_t used;',
            '    bool skip, have_seq, raw_unknown;',
            '    cs_link_counters_t stats;',
            '} cs_link_decoder_t;',
            'void cs_link_decoder_init(cs_link_decoder_t *decoder, bool raw_unknown);',
            'void cs_link_decoder_reset_sequence(cs_link_decoder_t *decoder);',
            'void cs_link_decoder_feed(cs_link_decoder_t *decoder, const uint8_t *data, size_t len, cs_link_receive_fn receive, void *context);',
            '#endif', '']
    write(ROOT/'core/cs_link.h', '\n'.join(out))
if __name__ == '__main__' and '--headers' in sys.argv:
    headers()

def literal(v):
    return '(-2147483647 - 1)' if v == -2147483648 else str(v)

def bytes_decl(name, data):
    return f'static const uint8_t {name}[] = {{'+(','.join(str(b) for b in data) or '0')+'};'

def fixtures():
    vectors=json.loads((ROOT.parents[1]/'tests/link_vectors.json').read_text())
    # Differential noise campaign: expected acceptance and every counter come
    # from the unchanged Python streaming decoder, not from the C code.
    rng=random.Random(261026)
    for _ in range(40):
        wire=bytearray()
        for j in range(12):
            if j%3==0:
                wire.extend(rng.randbytes(rng.randrange(0,300)))
                wire.append(0)
            good=bytearray(link.encode(link.Ack(cmd_seq=j),rng.randrange(256)))
            if j%4==1: good[rng.randrange(len(good)-1)]^=rng.randrange(1,256)
            elif j%4==2: del good[rng.randrange(len(good)-1)]
            wire.extend(good)
        decoder=link.Decoder()
        decoded=decoder.feed(wire)
        vectors['streams'].append({'wire':wire.hex(), 'decoded':[[type(x).__name__,x.seq] for x in decoded], 'counters':decoder.stats()})
    out=['/* GPL-3.0-only. Generated oracle fixtures; do not hand edit. */']
    for i,v in enumerate(vectors['messages']):
        cls=link.MESSAGES[v['id']]; n=cname(cls)
        out += [f'static void message_{i}(void) {{',f'cs_link_{n}_t a = {{0}}, b = {{0}};', 'uint8_t payload[249], wire[255];']
        for field,value in v['raw'].items():
            if isinstance(value,list): out += [f'a.{field}[{j}] = {literal(x)};' for j,x in enumerate(value)]
            else: out += [f'a.{field} = {literal(value)};']
        out += [bytes_decl('p',bytes.fromhex(v['payload'])),bytes_decl('w',bytes.fromhex(v['wire'])),
                f'CS_CHECK_EQ_U(cs_link_pack_{n}(payload, sizeof payload, &a), sizeof p);',
                'CS_CHECK_MEM(payload,p,sizeof p);',
                f'CS_CHECK(cs_link_unpack_{n}(&b,p,sizeof p));',
                f'CS_CHECK(!cs_link_unpack_{n}(&b,p,sizeof p-1));',
                f'CS_CHECK_EQ_U(cs_link_pack_{n}(payload, sizeof p-1, &a), 0);']
        for field,value in v['raw'].items():
            if isinstance(value,list): out += [f'CS_CHECK_EQ(b.{field}[{j}], a.{field}[{j}]);' for j in range(len(value))]
            else: out += [f'CS_CHECK_EQ(b.{field},a.{field});']
        out += [f'CS_CHECK_EQ_U(cs_link_frame(wire,sizeof wire,{v["id"]},{v["seq"]},p,sizeof p),sizeof w);','CS_CHECK_MEM(wire,w,sizeof w);','}']
    out += ['static void crc_and_cobs(void) { uint8_t outbuf[1024], back[1024]; size_t n, m;']
    for v in vectors['crc16']:
        data=bytes.fromhex(v['data']);out += ['{',bytes_decl('d',data),f'CS_CHECK_EQ_U(cs_crc16(d,{len(data)},0xffff),{v["crc"]});','}']
    for v in vectors['cobs']:
        data=bytes.fromhex(v['data']); stuffed=bytes.fromhex(v['stuffed'])
        out += ['{',bytes_decl('d',data),bytes_decl('w',stuffed),f'CS_CHECK(cs_cobs_encode(d,{len(data)},outbuf,sizeof outbuf,&n));',f'CS_CHECK_EQ_U(n,{len(stuffed)});','CS_CHECK_MEM(outbuf,w,n);',f'CS_CHECK(cs_cobs_decode(w,sizeof w,back,sizeof back,&m));',f'CS_CHECK_EQ_U(m,{len(data)});','CS_CHECK_MEM(back,d,m);','}']
    out += ['}']
    for i,v in enumerate(vectors['streams']):
        out += [f'static void stream_{i}(void) {{',bytes_decl('w',bytes.fromhex(v['wire'])), 'size_t chunk; for(chunk=1;chunk<=sizeof w;chunk+=17) { cs_link_decoder_t d; size_t at; received=0; cs_link_decoder_init(&d,false);',
                'for(at=0;at<sizeof w;at+=chunk) cs_link_decoder_feed(&d,w+at,(sizeof w-at<chunk)?sizeof w-at:chunk,capture,NULL);',f'CS_CHECK_EQ_U(received,{len(v["decoded"])});']
        for j,(name,seq) in enumerate(v['decoded']): out += [f'CS_CHECK_EQ_U(ids[{j}],{getattr(link,name).ID});',f'CS_CHECK_EQ_U(seqs[{j}],{seq});']
        for k in link.COUNTERS: out += [f'CS_CHECK_EQ_U(d.stats.{"short_frame" if k=="short" else k},{v["counters"].get(k,0)});']
        out += ['}}']
    for i,model in enumerate(vectors['imu_model']):
        out += [f'static void imu_model_{i}(void) {{ cs_imu_t s; cs_link_imu_t msg; cs_imu_init(&s,{model["epoch"]});']
        for step,snapshot in zip(model['steps'],model['snapshots']):
            for rate,force in step['samples']:
                out += ['{ const float r[3] = {'+','.join(str(x)+'f' for x in rate)+'}; const float f[3] = {'+','.join(str(x)+'f' for x in force)+'};',f'CS_CHECK(cs_imu_add_sample(&s,r,f,{model["h"]},0));','}']
            out += [f'CS_CHECK(cs_imu_snapshot(&s,{step["tick"]},&msg));',f'CS_CHECK_EQ_U(msg.flags,{snapshot["flags"]});','CS_CHECK_EQ_U(msg.n_gyro,10); CS_CHECK_EQ_U(msg.n_accel,10);']
            for k in range(4): out += [f'CS_CHECK_NEAR(msg.q[{k}],{snapshot["q"][k]},{model["q_tolerance"]});']
            for k in range(3): out += [f'CS_CHECK_NEAR(msg.u[{k}],{snapshot["u"][k]},{model["u_tolerance"]});']
        out += ['}']
    out += ['static void telemetry_vectors(void) { cs_telemetry_t t, d; uint8_t b[40];']
    cases=[telemetry.Telemetry(),telemetry.Telemetry(seq=65535,t_ms=0xffffffff,state=7,flags=0xf0,pressure_pa=101325,temperature_c=-2.125,altitude_m=1.25,gps_e=-1.25,gps_n=1e6,gps_u=-1e6,trn_e=23.4,trn_n=-12.3,trn_major_m=2.5,trn_minor_m=.85,trn_angle_rad=-.1,protection_m=100,landmarks=255,pred_e=99,pred_n=-87,time_to_go_s=40,pred_major_m=2.25,pred_minor_m=.75,pred_angle_rad=3.141592653589793,battery_v=3.875)]
    for case in cases:
        raw=list(telemetry._BODY.unpack(telemetry.encode(case)[:38]))[1:]
        fields=list(case.as_dict());out += ['{',bytes_decl('want',telemetry.encode(case)),'cs_telemetry_init(&t);']
        for name,val in zip(fields,raw[:2]+[raw[2]&15,raw[2]&240]+raw[3:]): out += [f't.{name} = {literal(val)};']
        out += ['CS_CHECK_EQ_U(cs_telemetry_encode(b,sizeof b,&t),40); CS_CHECK_MEM(b,want,40);','CS_CHECK(cs_telemetry_decode(&d,want,40)); CS_CHECK_EQ_U(cs_telemetry_encode(b,sizeof b,&d),40); CS_CHECK_MEM(b,want,40);','}']
    out += ['}']
    write(ROOT/'tests/core_a_vectors.inc','\n'.join(out)+'\n')
    names=['crc_and_cobs']+[f'message_{i}' for i in range(len(vectors['messages']))]+[f'stream_{i}' for i in range(len(vectors['streams']))]+['imu_model_0','telemetry_vectors']
    write(ROOT/'tests/test_core_a_vectors.c','''/* GPL-3.0-only. Python golden vectors, fragmented/malformed streams and IMU model.
 * CS_SOURCES: cs_crc16.c cs_cobs.c cs_link.c cs_imu.c cs_telemetry.c */
#include "cs_test.h"
#include "cs_crc16.h"
#include "cs_cobs.h"
#include "cs_link.h"
#include "cs_imu.h"
#include "cs_telemetry.h"
static size_t received;
static uint8_t ids[256], seqs[256];
static void capture(void *ctx,uint8_t id,uint8_t seq,const uint8_t *p,size_t n) {
    (void)ctx; (void)p; (void)n;
    if(received<256) { ids[received]=id;seqs[received]=seq; } received++;
}
#include "core_a_vectors.inc"
CS_MAIN('''+','.join(names)+')\n')

if __name__ == '__main__':
    fixtures()
