"""GPL-3.0-only. Run reproducible attachment audit and synthetic experiments.

Usage: python sim/maths/run_feasibility.py [--source-pdf PATH] [--skip-baseline]
All numerical evidence requires only NumPy. PDF indexing uses optional pypdf.
Outputs and raw synthetic inputs are retained in results/attachment_audit/.
"""
import argparse
import csv
import hashlib
import io
import json
import math
import platform
from pathlib import Path
import re
import time
import tracemalloc
import unittest
import numpy as np
from models import *
from feasibility import *
from test_physics_extra import lora_airtime
import run_verification as baseline

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent/'results'/'attachment_audit'
SEED=20261001


def write_csv(path, records):
    keys=list(dict.fromkeys(k for row in records for k in row))
    with path.open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=keys); writer.writeheader(); writer.writerows(records)


def rendered_navigation():
    """Paired synthetic scenes, not representative farmland or first-fix evidence."""
    rng=np.random.default_rng(SEED); rows=[]; matches=[]; frames={}; scenes=[]; noises=[]; result={}
    scenarios=[('flat',{}),('photometric',{'gain':.65,'offset':30}),
               ('relief_2m',{'relief':2.}),('relief_5m',{'relief':5.}),
               ('spin_blur_1ms',{'blur':True}),('periodic_texture',{'repetitive':True})]
    for trial in range(24):
        height=[80.,150.,300.][trial%3]
        pos=np.r_[rng.uniform(-10,10,2),height]
        r=np.diag([1.,-1.,-1.])@rotation(rng.normal(size=3)*np.array([.06,.06,.3]))
        pred=pos+np.r_[rng.normal(size=2)*1.5,rng.normal()*.3]
        pr=r@rotation(rng.normal(size=3)*np.array([.004,.004,.002]))
        xy=np.array([(x,y) for x in (-.4*height,0.,.4*height) for y in (-.23*height,0.,.23*height)])
        landmarks=np.c_[xy+pos[:2],np.zeros(9)]
        shared_noise=(rng.normal(size=(400,640))*2.).astype(np.float32)
        scenes.append(np.r_[pos,r.ravel(),pred,pr.ravel()]); noises.append(shared_noise)
        for name,settings in scenarios:
            start=time.perf_counter()
            kwargs={k:v for k,v in settings.items() if k in ('relief','repetitive')}
            if settings.get('blur'):
                image=sum(render_image(pos,r@rotation([0,0,2*math.pi*t]),**kwargs)
                          for t in np.linspace(-.0005,.0005,9))/9
            else: image=render_image(pos,r,**kwargs)
            image=settings.get('gain',1.)*image+settings.get('offset',0.)+shared_noise
            points,observed,records=image_matches(image,landmarks,pred,pr,settings.get('repetitive',False))
            row={'scenario':name,'trial':trial,'height_m':height,'matches':len(points),'fixed':False,
                 'initial_horizontal_error_m':float(np.linalg.norm(pred[:2]-pos[:2]))}
            for record in records: matches.append(dict(scenario=name,trial=trial,**record))
            if len(points)>=4:
                try:
                    fitted,attitude,cov=fit_pose(points,observed,pred,pr)
                    error=fitted[:2]-pos[:2]
                    # This local covariance is deliberately conditional on correct
                    # matches and plane terrain, not an asserted flight PL.
                    mahal=float(error@np.linalg.solve(cov[:2,:2],error))
                    row.update(fixed=True,error_x_m=float(error[0]),error_y_m=float(error[1]),
                               radial_error_m=float(np.linalg.norm(error)),
                               attitude_error_deg=float(np.linalg.norm(rotation_log(r.T@attitude))*180/math.pi),
                               local95_contains=bool(mahal<=-2*math.log(.05)),
                               mahalanobis2=mahal,local_axis_sd_m=float(np.sqrt(np.trace(cov[:2,:2])/2)))
                except (ValueError,np.linalg.LinAlgError): row['fit_failure']=True
            row['host_elapsed_s']=time.perf_counter()-start; rows.append(row)
            if trial==0:
                frames[name]=np.clip(image,0,255).astype(np.uint8)
                frames[name+'_true_position']=pos; frames[name+'_true_rotation']=r
                frames[name+'_initial_position']=pred; frames[name+'_initial_rotation']=pr
                frames[name+'_landmarks']=landmarks
    for name,_ in scenarios:
        subset=[x for x in rows if x['scenario']==name]; fixed=[x for x in subset if x['fixed']]
        result[name]={'trials':len(subset),'fixes':len(fixed),'fix_rate':len(fixed)/len(subset),
                      'radial_rmse_m':float(np.sqrt(np.mean([x['radial_error_m']**2 for x in fixed]))) if fixed else None,
                      'local95_coverage':float(np.mean([x['local95_contains'] for x in fixed])) if fixed else None,
                      'mean_accepted_matches':float(np.mean([x['matches'] for x in subset]))}
    write_csv(OUT/'image_trials.csv',rows); write_csv(OUT/'image_matches.csv',matches)
    np.savez_compressed(OUT/'example_rendered_frames.npz',**frames)
    np.savez_compressed(OUT/'image_scene_inputs.npz',
                        scenes=np.array(scenes),noise_gray=np.array(noises),
                        columns=np.array(['true_pos3','true_rotation9','initial_pos3','initial_rotation9']))
    return {'scope':'640x400, focal320px, 24 independent poses per scenario, +/-8m local search; no coarse acquisition, no full nonlinear ESKF',
            'noise_gray_sd':2.,'assumed_peak_pixel_sd':.5,'scenarios':result}


def linear_navigation():
    """15-state local linear navigation at 100Hz, 5Hz known landmark updates.

    Same F/Q truth and estimator checks matched-model consistency. Actual image
    associations and nonlinear injection are tested separately; not combined.
    """
    rng=np.random.default_rng(SEED+1); n=1000; dt=.01
    r=np.diag([1.,-1.,-1.]); f,d=eskf_matrices(r,np.array([0.,0.,-G0]),np.zeros(3))
    phi,q=discretize(f,d,dt); root=psd_root(q)
    sd=np.r_[np.ones(3)*2,np.ones(3)*.3,np.ones(3)*math.radians(.5),np.ones(3)*.03,np.ones(3)*.002]
    p=np.diag(sd*sd); truth=sd[:,None]*rng.normal(size=(15,n)); mean=np.zeros_like(truth)
    rows=[]; snapshots={}; full_errors=[]
    for step in range(2857):
        if step:
            truth=phi@truth+root@rng.normal(size=(15,n))
            mean=phi@mean; p=sym(phi@p@phi.T+q)
        if step%20==0:
            height=300-10.5*step*dt
            if height>=80:
                points=np.array([(x,y,0.) for x in (-.4*height,0,.4*height) for y in (-.23*height,0,.23*height)])
                poseh=pose_jacobian(points,np.array([0.,0.,height]),r,focal=640.)
                h=np.zeros((19,15)); h[:18,:3]=poseh[:,:3]; h[:18,6:9]=poseh[:,3:]; h[18,2]=1
                v=np.diag(np.r_[np.full(18,.2**2),.5**2])
            else:
                # Assumed VO-labelled velocity channel; actual image differences
                # and their temporal correlation remain a separate experiment.
                h=np.zeros((3,15)); h[0,3]=h[1,4]=h[2,2]=1
                v=np.diag([.6**2,.6**2,.5**2])
            observation=h@truth+np.sqrt(np.diag(v))[:,None]*rng.normal(size=(len(v),n))
            mean,p,_=gaussian_update(mean,p,observation,h,v)
        if step in (0,1000,2000,2800,2856):
            error=truth-mean; nees=np.sum(error*np.linalg.solve(p,error),axis=0)
            herror=error[:2]; hp=p[:2,:2]; maha=np.sum(herror*np.linalg.solve(hp,herror),axis=0)
            rows.append({'time_s':step*dt,'height_m':300-10.5*step*dt,
                         'full_nees_mean':float(nees.mean()),'full_nees_standard_error':math.sqrt(30/n),
                         'horizontal_rmse_m':float(np.sqrt(np.mean(np.sum(herror*herror,axis=0)))),
                         'horizontal95_coverage':float(np.mean(maha<=-2*math.log(.05)))})
            snapshots[str(step)+'_truth']=truth.copy(); snapshots[str(step)+'_estimate']=mean.copy(); snapshots[str(step)+'_cov']=p.copy()
        if step%20==0: full_errors.append((truth-mean).astype(np.float32))
    np.savez_compressed(OUT/'linear_navigation_raw.npz',**snapshots,errors_5hz=np.array(full_errors))
    write_csv(OUT/'linear_navigation.csv',rows)
    return {'descents':n,'state_dimension':15,'scope':'matched local linear reference, perfect map associations above80m, independent0.6m/s horizontal velocity updates below80m; no actual image-to-ESKF coupling',
            'epochs':rows}


def atmosphere_uncertainty():
    rng=np.random.default_rng(SEED+2); n=20000; raw={}; result={}
    for label,wind_difference in [('resolved_shear',1.),('weak_shear',.1)]:
        inputs=np.array([.1,20.,wind_difference,.0,290.])+rng.normal(size=(n,5))*np.array([.05,.1,.2,.2,.1])
        q,h,a,b,theta=inputs.T; ri=G0/theta*q*h/(a*a+b*b)
        raw[label+'_inputs']=inputs; raw[label+'_Ri']=ri
        result[label]={'assumed_delta_wind_mps':wind_difference,'Ri_quantiles_2.5_50_97.5':np.quantile(ri,[.025,.5,.975]).tolist(),
                       'negative_probability':float(np.mean(ri<0)),'above_quarter_probability':float(np.mean(ri>.25)),
                       'scope':'sample quantiles only; near-zero shear gives heavy tails, not a reliable categorical stability verdict'}
    z=np.linspace(10,80,30); x=np.c_[np.log(z),np.ones(30)]; truth=np.array([.75,-.75*math.log(.1)])
    cov=.15**2*.8**abs(np.subtract.outer(np.arange(30),np.arange(30)))
    cb=np.linalg.solve(x.T@np.linalg.solve(cov,x),np.eye(2)); gain=cb@x.T@np.linalg.solve(cov,np.eye(30))
    observations=x@truth[:,None]+psd_root(cov)@rng.normal(size=(30,n)); fitted=gain@observations
    logz=-fitted[1]/fitted[0]; raw['roughness_fits']=fitted; raw['roughness_observations']=observations
    result['roughness']={'true_z0_m':.1,'z0_quantiles_m':np.exp(np.quantile(logz,[.025,.5,.975])).tolist(),
                         'assumptions':'neutral, known heights/displacement, AR1=.8 SD=.15m/s, no temporal meteorological evolution'}
    height_speed=np.array([300.,10.5])+rng.normal(size=(n,2))*np.array([.5,.3])
    time_values=height_speed[:,0]/height_speed[:,1]; raw['time_height_speed']=height_speed
    result['touchdown_time']={'nominal_s':300/10.5,'empirical_sd_s':float(time_values.std()),
                             'linear_sd_s':math.sqrt(.5**2/10.5**2+300**2*.3**2/10.5**4),
                             'quantiles_s':np.quantile(time_values,[.025,.5,.975]).tolist()}
    t=np.arange(0,30.0001,.01); theta_amplitude=math.radians(15); length=1.5; omega=math.sqrt(G0/length); tau=10.5/G0
    swing_velocity=length*theta_amplitude*omega*np.cos(omega*t)
    false_wind=swing_velocity-tau*length*theta_amplitude*omega**2*np.sin(omega*t)
    raw['swing_time_s']=t; raw['swing_velocity_mps']=swing_velocity; raw['swing_false_wind_mps']=false_wind
    result['shared_swing_wind_bias']={'assumptions':'linear15deg swing,1.5m fixed pivot,isotropic tau; sensors share physical motion, perfect derivatives',
                                    'wind_rmse_mps':float(np.sqrt(np.mean(false_wind**2))),
                                    'image_minus_GPS_shared_bias_mps':0.,
                                    'meaning':'even perfect agreement between two sensor inversions can hide this common wind error'}
    thermistor_rows=[]
    for tau_t in (.1,1.,2.):
        air=285+.008*10.5*t
        sensed=air-tau_t*.008*10.5*(1-np.exp(-t/tau_t))
        raw['temperature_tau'+str(tau_t)]=sensed
        thermistor_rows.append({'tau_s':tau_t,'lag_length_m':10.5*tau_t,'bias_after10s_K':float((sensed-air)[1000])})
    result['thermistor']=thermistor_rows; raw['temperature_air_K']=air
    np.savez_compressed(OUT/'atmosphere_raw.npz',**raw)
    return result


def covariance_selection():
    rng=np.random.default_rng(SEED+3); z=np.linspace(300,10,70); warp_att=np.log(z)/.5
    models={}
    for kind in ('exponential','transverse'):
        for scale in ('constant','attached','saturating'):
            q=z/50 if scale=='constant' else warp_att+(z/100 if scale=='saturating' else 0.)
            delta=abs(q[:,None]-q[None,:]); models[kind+'_'+scale]=np.exp(-delta) if kind=='exponential' else transverse(delta)
    k=lag_matrix(len(z),abs(z[1]-z[0]),10.5**2/G0); noise=.6**2*np.eye(len(z)); scores={}; matrices={}
    for name,prior in models.items():
        sy=k@prior@k.T+noise; matrices[name]=sy
    truth=models['transverse_attached']; w=psd_root(truth)@rng.normal(size=(70,1000)); y=k@w+.6*rng.normal(size=w.shape)
    for name,sy in matrices.items():
        scores[name]=-.5*(70*math.log(2*math.pi)+np.linalg.slogdet(sy)[1]+np.sum(y*np.linalg.solve(sy,y),axis=0))
    values=np.array(list(scores.values())); chosen=values.argmax(axis=0); names=list(scores)
    np.savez_compressed(OUT/'covariance_selection_raw.npz',wind=w,observations=y,scores=values,heights=z,model_names=np.array(names))
    return {'independent_profiles':1000,'true_model':'transverse_attached','selection_counts':{name:int(np.sum(chosen==i)) for i,name in enumerate(names)},
            'scope':'six fixed known-parameter candidate priors, velocity noise0.6, likelihood comparison; no fitted-model Bayes factors or proof of physical eddy scaling'}


def processing_benchmarks():
    rng=np.random.default_rng(SEED+4); rows=[]; raw={}
    for name,imsize,tsize,landmarks in [('fine',96,32,50),('coarse',364,64,15)]:
        image=rng.normal(size=(imsize,imsize)); template=rng.normal(size=(tsize,tsize))
        ncc_fft(image,template)
        durations=[]
        for _ in range(20):
            t=time.perf_counter(); ncc_fft(image,template); durations.append(time.perf_counter()-t)
        tracemalloc.start(); ncc_fft(image,template); _,peak=tracemalloc.get_traced_memory(); tracemalloc.stop()
        valid=imsize-tsize+1; old=valid**2*tsize**2*8
        rows.append({'stage':name,'image_side_px':imsize,'template_side_px':tsize,
                     'landmarks_per_frame':landmarks,'median_single_ms':1000*float(np.median(durations)),
                     'p95_single_ms':1000*float(np.quantile(durations,.95)),
                     'serial_correlations_ms':1000*landmarks*float(np.median(durations)),
                     'tracemalloc_peak_bytes':peak,'old_window_array_bytes':old})
        raw[name+'_durations_s']=durations
    queues=[]
    for service,workers in [(.15,1),(.20,1),(.25,1),(.25,2)]:
        wait,latency=queue_simulation(np.full(150,service),workers=workers)
        queues.append({'assumed_service_s':service,'workers':workers,'last_wait_s':float(wait[-1]),'last_latency_s':float(latency[-1])})
        raw[f'queue_{service}_{workers}_wait']=wait; raw[f'queue_{service}_{workers}_latency']=latency
    airtime=[]
    for sf in range(7,13):
        de=int(2**sf/125000>.016); packet=lora_airtime(40,sf,125000,1,1,0,de,8)
        airtime.append({'sf':sf,'packet_s':packet,'occupancy_at_1Hz_percent':100*packet,
                        'interval_for_assumed1percent_s':packet/.01})
    raw['radio_packet_seconds']=np.array([a['packet_s'] for a in airtime])
    np.savez_compressed(OUT/'processing_raw.npz',**raw); write_csv(OUT/'benchmarks.csv',rows); write_csv(OUT/'queues.csv',queues); write_csv(OUT/'radio.csv',airtime)
    return {'host':platform.platform(),'NCC':rows,'queue_scenarios':queues,'radio_sweep':airtime,
            'limits':'host timing is measured; not an H618 benchmark. 1% is an illustrative arithmetic budget, not a regulatory assertion. NCC excludes rectification, capture, logging, filtering, and resource contention.'}


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--source-pdf',type=Path); parser.add_argument('--skip-baseline',action='store_true'); args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    start=time.perf_counter(); stream=io.StringIO()
    suite=unittest.defaultTestLoader.discover(str(Path(__file__).parent),pattern='test*.py')
    tests=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (OUT/'tests.txt').write_text(stream.getvalue(),encoding='utf-8')
    if not tests.wasSuccessful(): raise SystemExit('Numerical checks failed; experiments not run')
    report={'date':'2026-10-01','label':'SYNTHETIC except explicitly measured host timings','seed':SEED,'tests':tests.testsRun,
            'python':platform.python_version(),'numpy':np.__version__}
    for name,experiment in [('images',rendered_navigation),('linear_navigation',linear_navigation),
                             ('atmosphere',atmosphere_uncertainty),('model_selection',covariance_selection),('processing',processing_benchmarks)]:
        print('Running '+name,flush=True); report[name]=experiment()
    if not args.skip_baseline:
        print('Rerunning existing2000-descent scenarios',flush=True)
        baseline.OUT=OUT/'baseline'; baseline.OUT.mkdir(exist_ok=True)
        # Original files are preserved. New output has the original seed/model date.
        # Retain its normal report without flooding the console with layer tables.
        from contextlib import redirect_stdout
        with (baseline.OUT/'run_log.txt').open('w',encoding='utf-8') as log:
            with redirect_stdout(log): baseline.main()
    if (OUT/'baseline'/'verification.json').exists():
        baseline.OUT=OUT/'baseline'
        original=json.loads((Path(__file__).parent/'results'/'verification.json').read_text())
        rerun=json.loads((baseline.OUT/'verification.json').read_text())
        report['baseline_reproduction']={'original_model_date':rerun['date'],'rerun_date':'2026-10-01',
            'matched_layer_RMS_delta_mps':rerun['scenarios']['matched']['TRN']['mean_20m_layer_rmse_mps_80_to_300']-original['scenarios']['matched']['TRN']['mean_20m_layer_rmse_mps_80_to_300'],
            'original_tests':original['tests_run'],'current_tests':rerun['tests_run'],
            'scope':'baseline rerun retained in baseline/; --skip-baseline reuses that snapshot'}
    if args.source_pdf:
        from pypdf import PdfReader
        pdf=args.source_pdf; reader=PdfReader(pdf)
        text='\n\n'.join(f'=== PAGE {i+1} ===\n'+(page.extract_text() or '') for i,page in enumerate(reader.pages))
        (OUT/'source_text.txt').write_text(text,encoding='utf-8')
        report['source']={'path':str(pdf),'pages':len(reader.pages),'sha256':hashlib.sha256(pdf.read_bytes()).hexdigest()}
        from equation_coverage import inventory
        coverage=inventory(text,suite)
        write_csv(OUT/'equation_coverage.csv',coverage)
        report['equations_indexed']=len(coverage)
    report['elapsed_s']=time.perf_counter()-start
    (OUT/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    inputs=[p for p in Path(__file__).parent.glob('*.py')]
    inputs.extend([ROOT/'wiki'/'trn_secondary_mission_proposal.md',ROOT/'wiki'/'cansat_mathematics.tex'])
    manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    manifest.update({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.rglob('*') if p.is_file() and p.name!='manifest.json'})
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('linear_navigation','processing')},indent=2),flush=True)


if __name__=='__main__': main()
