"""GPL-3.0-only. Independent identities, limiting cases and numerical checks."""
import itertools
import math
import unittest
import numpy as np
from models import *


class Verification(unittest.TestCase):
    def test_lyapunov_and_covariance(self):
        np.testing.assert_allclose(F@P_INF+P_INF@F.T+B@B.T, 0, atol=1e-15)
        for r in [0, .1, 1, 2, 3, 10]:
            exact = math.exp(-r)*np.array([[1+r, r], [-r, 1-r]])
            np.testing.assert_allclose(expm(F*r), exact, atol=2e-14)
            self.assertAlmostEqual(C@P_INF@exact.T@C, transverse(r), places=14)

    def test_spectrum_fourier_integral(self):
        r = np.linspace(0, 35, 40001)
        for omega in [0., .3, 1., 3.]:
            # Two-sided Fourier transform of even g; independent quadrature.
            val = 2*np.trapezoid(transverse(r)*np.cos(omega*r), r)
            self.assertAlmostEqual(val, (1+3*omega**2)/(1+omega**2)**2, places=6)

    def test_warp_psd_ratio(self):
        z = np.geomspace(.5, 300, 80)
        for linf in [None, 100.]:
            self.assertGreater(np.linalg.eigvalsh(wind_cov(z, linf=linf)).min(), 0)
        self.assertAlmostEqual(wind_cov(np.array([10., 20.]))[0, 1], wind_cov(np.array([100., 200.]))[0, 1])
        with self.assertRaises(ValueError):
            warp([0.])

    def test_discretization_independent_ode(self):
        a, d = state_model(35., 11., 10.5)
        phi, q = discretize(a, d, 2.1)
        initial = np.concatenate([np.eye(4).reshape(-1), np.zeros(16)])
        def ode(t, x):
            p, v = x[:16].reshape(4, 4), x[16:].reshape(4, 4)
            return np.r_[(a@p).reshape(-1), (a@v+v@a.T+d).reshape(-1)]
        res = rk4(ode, initial, 2.1/500, 500)
        np.testing.assert_allclose(phi, res[:16].reshape(4, 4), atol=2e-12)
        np.testing.assert_allclose(q, res[16:].reshape(4, 4), atol=2e-12)
        p1,q1 = discretize(a,d,1.05)
        np.testing.assert_allclose(q, p1@q1@p1.T+q1, atol=1e-14)
        self.assertGreaterEqual(np.linalg.eigvalsh(q).min(), -1e-14)

    def test_lag_limits_and_constant_wind(self):
        n, ds, ell = 30, 2.1, 11.24
        k = lag_matrix(n, ds, ell)
        expected = 4*(1-np.exp(-ds*np.arange(1,n+1)/ell))
        np.testing.assert_allclose(k@np.full(n,4.), expected, atol=1e-14)
        np.testing.assert_allclose(lag_matrix(3,1,1e-6), np.eye(3))
        self.assertLess(np.max(lag_matrix(3,1,1e8)), 1.1e-8)

    def test_retrieval_alternative_precision_and_dfs(self):
        z = np.linspace(100, 2, 35)
        sa = wind_cov(z)
        k = lag_matrix(len(z), 2.8, 11.)
        se = np.eye(len(z))*.36
        gain, ak, post = retrieval(k,sa,se)
        precision = np.linalg.solve(sa,np.eye(len(z)))+k.T@np.linalg.solve(se,k)
        alternative = np.linalg.solve(precision,k.T@np.linalg.solve(se,np.eye(len(z))))
        np.testing.assert_allclose(gain,alternative,atol=2e-13)
        np.testing.assert_allclose(post,np.linalg.solve(precision,np.eye(len(z))),atol=2e-13)
        self.assertGreaterEqual(np.trace(ak),0)
        self.assertLessEqual(np.trace(ak),len(z))
        self.assertGreaterEqual(np.linalg.eigvalsh(sa-post).min(),-1e-12)

    def test_kalman_gp_and_rts_equivalence(self):
        z = np.arange(30., 0., -1)
        q = -warp(z)
        sa = wind_cov(z)
        rng = np.random.default_rng(8)
        y = rng.normal(size=len(z))
        noise=.16
        mean=np.zeros(2); p=P_INF.copy()
        mf=[]; pf=[]; mp=[]; pp=[]; phis=[]
        loglike=0.
        for i in range(len(z)):
            phi,qd = discretize(F,B@B.T,0 if i==0 else q[i]-q[i-1])
            mean=phi@mean; p=phi@p@phi.T+qd
            mp.append(mean.copy()); pp.append(p.copy()); phis.append(phi)
            mean,p,v,s=kalman_update(mean,p,y[i],C,noise)
            loglike-=.5*(math.log(2*math.pi*s.item())+v.item()**2/s.item())
            mf.append(mean.copy()); pf.append(p.copy())
        ms=mf.copy(); ps=pf.copy()
        for i in range(len(z)-2,-1,-1):
            j=np.linalg.solve(pp[i+1],phis[i+1]@pf[i]).T
            ms[i]=mf[i]+j@(ms[i+1]-mp[i+1])
            ps[i]=sym(pf[i]+j@(ps[i+1]-pp[i+1])@j.T)
        gain,_,post=retrieval(np.eye(len(z)),sa,np.eye(len(z))*noise)
        np.testing.assert_allclose(np.array(ms)@C,gain@y,atol=2e-13)
        np.testing.assert_allclose([C@p@C for p in ps],np.diag(post),atol=2e-13)
        sy=sa+np.eye(len(z))*noise
        direct=-.5*(len(z)*math.log(2*math.pi)+np.linalg.slogdet(sy)[1]+y@np.linalg.solve(sy,y))
        self.assertAlmostEqual(loglike,direct,places=11)

    def test_projection_jacobians_and_homography(self):
        pos=np.array([5.,-3.,100.]); r=rotation(np.array([.1,-.2,.3]))
        rcb=np.diag([1.,-1.,-1.]); point=np.array([30.,20.,0.])
        jp,jr=reprojection_jacobians(point,pos,r,rcb)
        eps=1e-5
        fdpos=[]; fdrot=[]
        for axis in np.eye(3):
            fdpos.append((project(point,pos+eps*axis,r,rcb)-project(point,pos-eps*axis,r,rcb))/(2*eps))
            fdrot.append((project(point,pos,r@rotation(eps*axis),rcb)-project(point,pos,r@rotation(-eps*axis),rcb))/(2*eps))
        np.testing.assert_allclose(jp,np.array(fdpos).T,atol=1e-8)
        np.testing.assert_allclose(jr,np.array(fdrot).T,rtol=1e-8,atol=1e-7)
        k=np.array([[640.,0,640],[0,640,400],[0,0,1]])
        rcw=rcb@r.T
        h=k@np.column_stack([rcw[:,0],rcw[:,1],-rcw@pos])
        pix=h@np.r_[point[:2],1]
        np.testing.assert_allclose(pix[:2]/pix[2],project(point,pos,r,rcb))

    def test_tilt_schur_information(self):
        def info(angles):
            h=np.column_stack([np.ones(len(angles)),1/np.cos(angles)**2])
            j=h.T@h
            return j[1,1]-j[0,1]**2/j[0,0]
        self.assertAlmostEqual(info(np.array([-.5,.5])),0.)
        self.assertGreater(info(np.array([-.7,0,.7])),info(np.array([-.1,0,.1])))

    def test_ncc_fft_vs_direct(self):
        rng=np.random.default_rng(1)
        image=rng.normal(size=(13,15)); t=image[5:9,7:12].copy()
        got=ncc_fft(image,t)
        expected=np.empty_like(got)
        for y,x in np.ndindex(got.shape):
            a=image[y:y+4,x:x+5]; a=a-a.mean(); b=t-t.mean()
            expected[y,x]=np.sum(a*b)/math.sqrt(np.sum(a*a)*np.sum(b*b))
        np.testing.assert_allclose(got,expected,atol=1e-14)
        self.assertEqual(np.unravel_index(np.argmax(got),got.shape),(5,7))
        np.testing.assert_allclose(ncc_fft(3*image+7,t),got,atol=2e-14)

    def test_parabola_and_ransac(self):
        peak=.23
        left,center,right=[2-(x-peak)**2 for x in [-1,0,1]]
        self.assertAlmostEqual((left-right)/(2*(left-2*center+right)),peak)
        n=math.ceil(math.log(.001)/math.log(1-.5**2))
        self.assertEqual(n,25)
        self.assertGreaterEqual(1-(1-.25)**n,.999)

    def test_terminal_descent_vs_ode(self):
        vt=10.5; t=5.
        result=rk4(lambda time,x:np.array([G0*(1-(x[0]/vt)**2),x[0]]),[0.,0.],.001,5000)
        analytic=[vt*math.tanh(G0*t/vt),vt**2/G0*math.log(math.cosh(G0*t/vt))]
        np.testing.assert_allclose(result,analytic,atol=1e-10)

    def test_nonlinear_horizontal_drag_linear_limit(self):
        vt=10.5; delta=.01; t=2.
        result=rk4(lambda time,x:-G0/vt*np.sqrt(1+(x/vt)**2)*x,[delta],.001,2000)
        self.assertLess(abs(result[0]-delta*math.exp(-G0*t/vt)),1e-8)

    def test_hypsometric_and_potential_temperature(self):
        rd=287.05; temp=288.; dz=300.; p1=100000.
        p2=p1*math.exp(-G0*dz/(rd*temp))
        self.assertAlmostEqual(rd*temp/G0*math.log(p1/p2),dz,places=10)
        # Constant theta: dry adiabat, T2/T1=(p2/p1)^(R/cp).
        t2=temp*(p2/p1)**(rd/1004.)
        self.assertAlmostEqual(t2*(p1/p2)**(rd/1004.),temp)

    def test_most_correction_derivative(self):
        def psi(q):
            if q>=0: return -5*q
            x=(1-16*q)**.25
            return 2*math.log((1+x)/2)+math.log((1+x*x)/2)-2*math.atan(x)+math.pi/2
        self.assertAlmostEqual(psi(0),0.)
        for q in [-1.,-.1,.1,1.]:
            ep=1e-6
            phi=1+5*q if q>0 else (1-16*q)**(-.25)
            self.assertAlmostEqual((psi(q+ep)-psi(q-ep))/(2*ep),(1-phi)/q,places=6)

    def test_greedy_normalized_submodularity(self):
        rng=np.random.default_rng(4)
        vectors=rng.normal(size=(7,3)); j=[np.outer(v,v) for v in vectors]
        def value(s): return np.linalg.slogdet(np.eye(3)+sum((j[i] for i in s),start=np.zeros((3,3))))[1]
        for s in [set(),{0},{0,1}]:
            for t in [s|{2},s|{2,3}]:
                for i in set(range(7))-t:
                    self.assertGreaterEqual(value(s|{i})-value(s)+1e-12,value(t|{i})-value(t))
        s=set()
        for _ in range(3):
            best=max(set(range(7))-s,key=lambda i:value(s|{i})-value(s)); s.add(best)
        optimum=max(value(set(c)) for c in itertools.combinations(range(7),3))
        self.assertGreaterEqual(value(s)/optimum,1-(1-1/3)**3)

    def test_tls_enumeration_global_bound(self):
        # Exact finite-switch verification for an affine similarity, not an SDP claim.
        u=np.array([0+0j,1+0j,0+1j,1+1j,2+0j]); m=(1.2+.3j)*u+2-1j
        m[-1]+=10+3j
        cap=1.
        candidates=[]
        for flags in itertools.product([False,True],repeat=len(u)):
            use=np.array(flags); design=np.column_stack([u[use],np.ones(use.sum())])
            coeff=np.linalg.lstsq(design,m[use],rcond=None)[0] if use.any() else np.zeros(2,dtype=complex)
            residual=abs(m-coeff[0]*u-coeff[1])**2
            switched=residual[use].sum()+(~use).sum()*cap
            candidates.append((switched,coeff))
        cost,coeff=min(candidates,key=lambda v:v[0])
        self.assertAlmostEqual(cost,1.)
        np.testing.assert_allclose(coeff,[1.2+.3j,2-1j],atol=1e-12)

    def test_shrinkage_sufficient_condition(self):
        # Valid Gaussian A,B covariance: removing B can INCREASE variance.
        p=np.array([[1.,-.2],[-.2,.01+.04]])
        self.assertGreater(np.linalg.eigvalsh(p).min(),0.)
        self.assertLess(np.array([1.,1.])@p@np.array([1.,1.]),p[0,0])
        # Constant-scale unconditioned transverse prior has positive integral covariance.
        for h in np.geomspace(.001,100,50):
            self.assertGreater(.5*(1+(h-1)*math.exp(-h)),0.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
