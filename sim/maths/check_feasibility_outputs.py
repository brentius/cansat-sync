"""GPL-3.0-only. Audit retained results; do not confuse file QA with new evidence."""
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np

OUT=Path(__file__).parent/'results'/'attachment_audit'
ROOT=Path(__file__).resolve().parents[2]


def main():
    r=json.loads((OUT/'summary.json').read_text())
    rows=list(csv.DictReader((OUT/'equation_coverage.csv').open(encoding='utf-8')))
    assert len(rows)==len(set(x['equation'] for x in rows))==249
    assert r['tests']>=76
    with np.load(OUT/'image_scene_inputs.npz') as raw:
        assert raw['scenes'].shape==(24,24)
        assert raw['noise_gray'].shape==(24,400,640)
        assert np.isfinite(raw['scenes']).all()
    trials=list(csv.DictReader((OUT/'image_trials.csv').open(encoding='utf-8')))
    matches=list(csv.DictReader((OUT/'image_matches.csv').open(encoding='utf-8')))
    assert len(trials)==144 and len(matches)==144*9
    for name in ('flat','relief_2m','periodic_texture'):
        assert r['images']['scenarios'][name]['trials']==24
    assert r['images']['scenarios']['flat']['fixes']>=20
    assert r['images']['scenarios']['relief_2m']['radial_rmse_m']>10*r['images']['scenarios']['flat']['radial_rmse_m']
    for epoch in r['linear_navigation']['epochs']:
        assert abs(epoch['full_nees_mean']-15)<5*math.sqrt(30/1000)
        assert abs(epoch['horizontal95_coverage']-.95)<5*math.sqrt(.95*.05/1000)
    assert sum(r['model_selection']['selection_counts'].values())==1000
    assert abs(r['atmosphere']['touchdown_time']['empirical_sd_s']-r['atmosphere']['touchdown_time']['linear_sd_s'])<.02
    baseline=json.loads((OUT/'baseline'/'verification.json').read_text())
    old=json.loads((OUT.parent/'verification.json').read_text())
    assert baseline['scenarios']==old['scenarios']
    for benchmark in r['processing']['NCC']:
        assert benchmark['tracemalloc_peak_bytes']<benchmark['old_window_array_bytes']/10
    for path in OUT.glob('*.npz'):
        with np.load(path,allow_pickle=False) as raw:
            for key in raw.files:
                array=raw[key]
                if np.issubdtype(array.dtype,np.number): assert np.isfinite(array).all(),(path,key)
    manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')}
    for p in [ROOT/'wiki'/'cansat_mathematics.tex',ROOT/'wiki'/'trn_secondary_mission_proposal.md',
              ROOT/'wiki'/'mathematics_simulation_audit.md',ROOT/'BOTS'/'PLANS'/'STATUS.md',ROOT/'BOTS'/'PLANS'/'DECISIONS.md']:
        if p.exists(): manifest[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    manifest.update({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.rglob('*') if p.is_file() and p.name not in ('manifest.json','output_checks.json')})
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    result={'passed':True,'equations_indexed':249,'image_trials':144,'image_match_records':1296,
            'baseline_scenario_reproduction':'exact equality','finite_raw_arrays':True,
            'linear15state_consistency':'all five comparable epochs within five standard errors',
            'scope':'data/file/regression QA, not additional independent experimental trials'}
    (OUT/'output_checks.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__': main()
