"""GPL-3.0-only. Reproducible synthetic experiments; NO measured flight data.

Run from any working directory: python sim/maths/run_verification.py
Outputs are kept beside this script in results/. Requires NumPy, not SciPy.
"""
import csv
import io
import json
import math
from pathlib import Path
import platform
import unittest
import numpy as np
from models import *
import test_models

OUT = Path(__file__).resolve().parent / 'results'
SEED = 20260930
N = 2000


def simulated_descent(ell_ratio=1., correlation=0., seed=SEED, floor=1., refinement=1, correct_correlation=False):
    """Two independent horizontal axes, 300m to 0m, 5Hz measurements.

    Synthetic mean 5m/s per axis, sigma1m/s, L=max(.5*z,.5*floor),
    vd10.5m/s. A regularized scale at the ground is a scenario, not a
    claimed physical boundary condition. Initial position std2m each axis.
    Shared prior and Gaussian velocity noise .1(GPS),.6(TRN) m/s.
    Noise is independent of the wind. Optional TRN AR1 tests misspecification.
    """
    rng = np.random.default_rng(seed)
    vd = 10.5
    ell = vd**2/G0
    heights = np.r_[np.arange(300., 0., -vd/5), 0.]
    steps = []
    true_steps = []
    for top,bottom in zip(heights[:-1],heights[1:]):
        ds = top-bottom
        pm=np.eye(4); qm=np.zeros((4,4)); bm=np.zeros(4)
        pt=pm.copy(); qt=qm.copy(); bt=bm.copy()
        # Midpoint coefficient freezing with refinement for convergence testing.
        for j in range(refinement):
            mid=top-(j+.5)*ds/refinement
            length=.5*max(mid,floor)
            phi,q,b=step_model(length,ell,vd,ds/refinement,mu=5.)
            bm=phi@bm+b; qm=phi@qm@phi.T+q; pm=phi@pm
            phi,q,b=step_model(length,ell*ell_ratio,vd,ds/refinement,mu=5.)
            bt=phi@bt+b; qt=phi@qt@phi.T+q; pt=phi@pt
        steps.append((pm,sym(qm),bm)); true_steps.append((pt,sym(qt),bt))
    a,d=state_model(150.,ell,vd)
    p0=np.zeros((4,4)); p0[:3,:3]=stationary_cov(a[:3,:3],d[:3,:3]); p0[3,3]=4.
    at,dt=state_model(150.,ell*ell_ratio,vd)
    ptrue=p0.copy(); ptrue[:3,:3]=stationary_cov(at[:3,:3],dt[:3,:3])
    initial=np.array([0.,0.,5.,0.])
    truth=np.empty((len(heights),4,2*N))
    truth[0]=initial[:,None]+psd_root(ptrue)@rng.normal(size=(4,2*N))
    for i,(phi,q,b) in enumerate(true_steps):
        truth[i+1]=phi@truth[i]+b[:,None]+psd_root(q)@rng.normal(size=(4,2*N))
    # Propagators to touchdown from every altitude, computed independently of updates.
    pend=np.zeros((len(heights),4,4)); qend=pend.copy(); bend=np.zeros((len(heights),4))
    pend[-1]=np.eye(4)
    for i in range(len(steps)-1,-1,-1):
        phi,q,b=steps[i]
        pend[i]=pend[i+1]@phi
        qend[i]=sym(pend[i+1]@q@pend[i+1].T+qend[i+1])
        bend[i]=pend[i+1]@b+bend[i+1]
    models={}
    saved=[]
    for name,noise in [('GPS',.1),('TRN',.6),('prior',None)]:
        m=np.tile(initial[:,None],(1,2*N)); p=p0.copy()
        augmented=correct_correlation and name=='TRN'
        filter_steps=steps
        if augmented:
            m=np.vstack([m,np.zeros((1,2*N))])
            p=np.pad(p,((0,1),(0,1))); p[-1,-1]=noise**2
            filter_steps=[augment_ar1_step(phi,q,b,correlation,noise) for phi,q,b in steps]
        means=[]; covs=[]; predmeans=[]; predcovs=[]
        ar=rng.normal(size=2*N)
        for i,z in enumerate(heights):
            if i:
                phi,q,b=filter_steps[i-1]; m=phi@m+b[:,None]; p=sym(phi@p@phi.T+q)
            predmeans.append(m.copy()); predcovs.append(p.copy())
            if noise is not None:
                rho=correlation if name=='TRN' else 0.
                if i: ar=rho*ar+math.sqrt(1-rho*rho)*rng.normal(size=2*N)
                y=truth[i,2]+noise*ar
                h=[0,0,1,0,1] if augmented else [0,0,1,0]
                m,p,_,_=kalman_update(m,p,y,h,0. if augmented else noise**2)
            means.append(m.copy()); covs.append(p.copy())
        sm,sp,smoother_gains=rts_smooth(means,covs,predmeans,predcovs,[step[0] for step in filter_steps])
        wtrue=5.+np.einsum('j,kjn->kn',C,truth[:,:2])
        what=5.+np.einsum('j,kjn->kn',C,np.array(sm)[:,:2])
        err=what-wtrue
        layer=[]; layer_statistics=[]
        output=np.r_[C,np.zeros(len(p)-2)]
        for low in np.arange(80.,300.,20.):
            mask=(heights>low)&(heights<=low+20.)
            le=err[mask].mean(axis=0)
            layer.append(float(np.sqrt(np.mean(le**2))))
            indices=np.flatnonzero(mask); weights=np.ones(len(indices))/len(indices)
            variance=layer_posterior_variance(indices,weights,output,sp,smoother_gains)
            independent_variance=sum(output@sp[i]@output for i in indices)/len(indices)**2
            layer_statistics.append({'bottom_m':float(low),'samples':len(indices),'empirical_component_rmse_mps':layer[-1],
                                     'posterior_component_sd_mps':float(math.sqrt(variance)),
                                     'diagonal_only_sd_mps':float(math.sqrt(independent_variance)),
                                     'coverage95':float(np.mean(abs(le)<=1.959963984540054*math.sqrt(variance)))})
        samples=[]
        for wanted in [300.,200.,100.,20.]:
            i=int(np.argmin(abs(heights-wanted)))
            predicted=pend[i]@means[i][:4]+bend[i][:,None]
            cov=sym(pend[i]@covs[i][:4,:4]@pend[i].T+qend[i])
            de=truth[-1,3]-predicted[3]
            radius2=de[:N]**2+de[N:]**2
            d2=radius2/cov[3,3]
            coverage=float(np.mean(d2<=-2*math.log(.05)))
            samples.append({'height_m':float(heights[i]),'radial_rms_m':float(np.sqrt(np.mean(radius2))),
                            'predicted_axis_sd_m':float(math.sqrt(cov[3,3])),
                            'coverage95':coverage,'mean_mahalanobis2':float(d2.mean())})
        models[name]={'wind_sample_rmse_mps':float(np.sqrt(np.mean(err**2))),
                      'mean_20m_layer_rmse_mps_80_to_300':float(np.mean(layer)),
                      'max_20m_layer_rmse_mps_80_to_300':float(np.max(layer)),
                      'layer_uncertainty':layer_statistics,
                      'landing':samples}
        # One example is explicitly synthetic, retained for reproducibility.
        for i,z in enumerate(heights):
            saved.append([name,float(z),float(wtrue[i,0]),float(what[i,0]),float(truth[i,2,0]),
                          float(np.sqrt(C@sp[i][:2,:2]@C))])
    return models,saved


def averaging_experiment():
    z=np.arange(298.95,1.,-2.1); n=len(z)
    sa=wind_cov(z); k=lag_matrix(n,2.1,10.5**2/G0)
    result={}
    for name,sd,rho in [('GPS',.1,0.),('TRN',.6,0.),('TRN_colored',.6,.85)]:
        se=sd**2*rho**abs(np.subtract.outer(np.arange(n),np.arange(n))) if rho else np.eye(n)*sd**2
        gain,ak,post=retrieval(k,sa,se)
        result[name]={'dfs':float(np.trace(ak)),'mean_posterior_sd_mps':float(np.sqrt(np.diag(post)).mean())}
    return result


def initial_velocity_experiment():
    """Independent dense Gaussian inversion including correlated u0 uncertainty."""
    rng=np.random.default_rng(SEED+7)
    z=np.arange(298.95,1.,-2.1); n=len(z)
    sa=wind_cov(z); ell=10.5**2/G0
    k=lag_matrix(n,2.1,ell); b=np.exp(-2.1*np.arange(1,n+1)/ell)
    cross=.7*sa[:,0]; initial_variance=.7**2+.8**2
    joint=np.block([[sa,cross[:,None]],[cross[None,:],np.array([[initial_variance]])]])
    # Construct u0 from w0 plus independent error, not from the estimator's posterior.
    true_w=psd_root(sa)@rng.normal(size=(n,2*N))
    u0=.7*true_w[0]+.8*rng.normal(size=2*N)
    se=np.eye(n)*.1**2
    y=k@true_w+b[:,None]*u0+.1*rng.normal(size=(n,2*N))
    fixed_gain,_,fixed_post=retrieval(k,sa,se)
    correct_gain,correct_post=retrieval_with_initial_velocity(k,b,sa,cross,initial_variance,se)
    rows=[]
    for name,gain,post in [('incorrect_fixed_u0',fixed_gain,fixed_post),('correct_joint_u0',correct_gain,correct_post)]:
        error=gain@y-true_w
        rows.append({'model':name,'first_sample_rmse_mps':float(np.sqrt(np.mean(error[0]**2))),
                     'first_sample_posterior_sd_mps':float(math.sqrt(post[0,0])),
                     'first_sample_coverage95':float(np.mean(abs(error[0])<=1.959963984540054*math.sqrt(post[0,0]))),
                     'all_samples_rmse_mps':float(np.sqrt(np.mean(error**2)))})
    return {'boundary_model':'u0=0.7*w_first+independent N(0,0.8^2), wind zero-mean perturbations',
            'joint_min_eigenvalue':float(np.linalg.eigvalsh(joint).min()),'models':rows}


def shrinkage_search():
    """Construct a counterexample INSIDE the warped transverse kernel family.

    Two 0.1m cells, remaining lower cell at z=1m and slab at z=exp(1.5)m;
    sigma1=100, sigma2=1. a=.5 gives eddy-coordinate separation3.
    A,B are their weighted point-quadrature drifts with vd=10.5.
    Extreme intensity contrast shows no general theorem for arbitrary sigma(z).
    Does not claim a realistic CanSat atmospheric profile.
    """
    z=np.array([1.,math.exp(1.5)])
    sigma=np.array([100.,1.])
    p=wind_cov(z,sigma=sigma)*(.1/10.5)**2
    before=np.ones(2)@p@np.ones(2)
    after=p[0,0]
    return {'heights_m':z.tolist(),'sigma_mps':sigma.tolist(),'minimum_cov_eigenvalue':float(np.linalg.eigvalsh(p).min()),
            'before_variance_m2':float(before),'after_variance_m2':float(after),
            'increase_m2':float(after-before),'scope':'finite-grid remaining-drift counterexample, arbitrary intensity'}


def nonlinear_mismatch():
    vd=10.5; tau=vd/G0; total=20.; dt=.001; count=int(total/dt)
    # Analytic smooth wind, vector drag evaluated independently with RK4.
    def wind(t): return 5+2*math.sin(.7*t)
    rows=[]
    for ratio in [.5,1.,2.]:
        def fun(t,u):
            rel=wind(t)-u[0]
            return np.array([ratio/tau*math.sqrt(1+(rel/vd)**2)*rel])
        u=np.array([5.]); squares=[]; wrong=[]
        for i in range(count):
            t=i*dt
            derivative=fun(t,u)[0]
            recovered=u[0]+tau*derivative
            calibrated=u[0]+tau/ratio*derivative
            wrong.append((recovered-wind(t))**2)
            squares.append((calibrated-wind(t))**2)
            k1=fun(t,u); k2=fun(t+dt/2,u+dt*k1/2)
            k3=fun(t+dt/2,u+dt*k2/2); k4=fun(t+dt,u+dt*k3)
            u+=dt*(k1+2*k2+2*k3+k4)/6
        rows.append({'lateral_drag_ratio':ratio,'assumed_isotropic_wind_rmse_mps':math.sqrt(np.mean(wrong)),
                     'calibrated_linear_wind_rmse_mps':math.sqrt(np.mean(squares))})
    return rows


def main():
    OUT.mkdir(exist_ok=True)
    stream=io.StringIO()
    suite=unittest.defaultTestLoader.discover(str(Path(__file__).resolve().parent),pattern='test*.py')
    tests=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (OUT/'tests.txt').write_text(stream.getvalue(),encoding='utf-8')
    if not tests.wasSuccessful(): raise SystemExit('verification failed')
    scenarios={}
    for name,kwargs in [('matched',{}),('lag_half',{'ell_ratio':.5}),('lag_double',{'ell_ratio':2.}),
                        ('correlated_vision_noise',{'correlation':.85}),
                        ('correlated_vision_noise_corrected',{'correlation':.85,'correct_correlation':True})]:
        print('Running '+name,flush=True)
        result,saved=simulated_descent(**kwargs)
        scenarios[name]=result
        if name=='matched':
            with (OUT/'synthetic_profile.csv').open('w',newline='',encoding='utf-8') as f:
                writer=csv.writer(f); writer.writerow(['source','height_m','true_wind_mps','smoothed_wind_mps','true_probe_velocity_mps','posterior_sd_mps']); writer.writerows(saved)
    # Covariance convergence and lower-boundary sensitivity (smaller ensemble is not used;
    # deterministic reported covariance is independent of ensemble size/seed).
    sensitivity=[]
    for floor,ref in [(1.,2),(1.,4),(.5,4),(2.,4)]:
        result,_=simulated_descent(floor=floor,refinement=ref)
        sensitivity.append({'floor_m':floor,'substeps':ref,'TRN_landing':result['TRN']['landing'],
                            'TRN_wind_rmse_mps':result['TRN']['wind_sample_rmse_mps']})
    report={'label':'SYNTHETIC; no flight data','date':'2026-09-30','seed':SEED,'independent_descents_per_scenario':N,
            'python':platform.python_version(),'numpy':np.__version__,'tests_run':tests.testsRun,'tests_passed':tests.wasSuccessful(),
            'assumptions':{'top_height_m':300,'descent_speed_mps':10.5,'camera_hz':5,'mean_wind_each_axis_mps':5,
                           'wind_sigma_mps':1,'length_scale':'0.5*max(z,floor_m)','initial_position_axis_sd_m':2,
                           'velocity_noise_gps_mps':.1,'velocity_noise_trn_mps':.6,'smoothing':'RTS, fixed known parameters',
                           'scope':'velocity wind retrieval and drift prediction; no image matcher or ESKF flight-performance claim'},
            'scenarios':scenarios,'averaging_kernels':averaging_experiment(),'nonlinear_drag':nonlinear_mismatch(),
            'initial_velocity':initial_velocity_experiment(),
            'shrinkage_counterexample':shrinkage_search(),'sensitivity':sensitivity}
    (OUT/'verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'tests':tests.testsRun,'matched':scenarios['matched'],'nonlinear_drag':report['nonlinear_drag'],
                      'averaging_kernels':report['averaging_kernels'],'counterexample':report['shrinkage_counterexample']},indent=2),flush=True)


if __name__=='__main__': main()
