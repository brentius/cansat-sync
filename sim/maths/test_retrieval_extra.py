"""GPL-3.0-only. Independent correlated-noise and boundary-condition checks."""
import math
import unittest
import numpy as np
from models import *


class RetrievalExtra(unittest.TestCase):
    def test_ar1_stationarity_and_colored_batch_equivalence(self):
        z=np.geomspace(30.,2.,18); q=-warp(z); n=len(z)
        sigma=.6; rho=.85
        sa=wind_cov(z)
        se=sigma**2*rho**abs(np.subtract.outer(np.arange(n),np.arange(n)))
        y=np.random.default_rng(3).normal(size=n)
        m=np.zeros(3); p=np.diag([.25,.25,sigma**2])
        mf=[]; pf=[]; mp=[]; pp=[]; phis=[]
        for i in range(n):
            if i:
                phix=expm(F*(q[i]-q[i-1]))
                qx=sym(P_INF-phix@P_INF@phix.T)
                phi,qd,b=augment_ar1_step(phix,qx,np.zeros(2),rho,sigma)
                # Verify its unconditional stationary covariance separately.
                prior=np.diag([.25,.25,sigma**2])
                np.testing.assert_allclose(phi@prior@phi.T+qd,prior,atol=1e-14)
                phis.append(phi)
                m=phi@m; p=sym(phi@p@phi.T+qd)
            mp.append(m.copy()); pp.append(p.copy())
            m,p,_,_=kalman_update(m,p,y[i],[1,math.sqrt(3),1],0.)
            mf.append(m.copy()); pf.append(p.copy())
        ms,ps,gains=rts_smooth(mf,pf,mp,pp,phis)
        direct_gain,_,direct_post=retrieval(np.eye(n),sa,se)
        h=np.r_[C,0.]
        np.testing.assert_allclose(np.array(ms)@h,direct_gain@y,atol=3e-13)
        got=np.array([[h@smoother_cross_cov(i,j,ps,gains)@h for j in range(n)] for i in range(n)])
        np.testing.assert_allclose(got,direct_post,atol=3e-13)
        np.testing.assert_allclose(got,got.T,atol=2e-14)
        # Layer variance from the smoother against the independently formed GP matrix.
        indices=list(range(5,13)); a=np.zeros(n); a[indices]=1/len(indices)
        actual=layer_posterior_variance(indices,a[indices],h,ps,gains)
        self.assertAlmostEqual(actual,a@direct_post@a,places=13)

    def test_initial_velocity_marginal_against_joint_gaussian(self):
        z=np.linspace(60,2,25); sa=wind_cov(z); n=len(z)
        k=lag_matrix(n,2.4,11.); b=np.exp(-2.4*np.arange(1,n+1)/11.)
        cross=.7*sa[:,0]; variance=.7**2+.8**2; se=np.eye(n)*.1**2
        gain,post=retrieval_with_initial_velocity(k,b,sa,cross,variance,se)
        joint=np.block([[sa,cross[:,None]],[cross[None,:],np.array([[variance]])]])
        forward=np.column_stack([k,b])
        full_gain,_,full_post=retrieval(forward,joint,se)
        np.testing.assert_allclose(gain,full_gain[:n],atol=3e-13)
        np.testing.assert_allclose(post,full_post[:n,:n],atol=3e-13)
        self.assertGreater(np.linalg.eigvalsh(joint).min(),0.)
        # Known initial perturbation, zero cross covariance, returns the original model.
        g0,p0=retrieval_with_initial_velocity(k,b,sa,np.zeros(n),0.,se)
        g1,_,p1=retrieval(k,sa,se)
        np.testing.assert_allclose(g0,g1,atol=1e-14)
        np.testing.assert_allclose(p0,p1,atol=1e-14)

    def test_layer_variance_equal_correlation_closed_form(self):
        n=10; rho=.85; sd=.6
        se=sd**2*rho**abs(np.subtract.outer(np.arange(n),np.arange(n)))
        a=np.ones(n)/n
        factor=1+2*sum((1-j/n)*rho**j for j in range(1,n))
        self.assertAlmostEqual(a@se@a,sd**2/n*factor,places=14)
        self.assertAlmostEqual(n/factor,1.5960995725957565,places=13)
        self.assertGreater(a@se@a,sd**2/n)

    def test_fixed_endpoint_covariance_conditioning_monotonic(self):
        # Joint covariance independently constructed as L L^T; state first two dims,
        # new measurements remaining three. Covers nonzero negative cross covariance.
        rng=np.random.default_rng(9); l=rng.normal(size=(5,7)); joint=l@l.T
        before=joint[:2,:2]
        cross=joint[:2,2:]; s=joint[2:,2:]
        after=sym(before-cross@np.linalg.solve(s,cross.T))
        self.assertGreater(np.linalg.eigvalsh(after).min(),0.)
        self.assertGreaterEqual(np.linalg.eigvalsh(before-after).min(),-1e-14)

    def test_ar1_invalid_parameters(self):
        for rho,sd in [(1.,.6),(-1.,.6),(0.,0.)]:
            with self.assertRaises(ValueError):
                augment_ar1_step(np.eye(2),np.eye(2),np.zeros(2),rho,sd)


if __name__=='__main__': unittest.main(verbosity=2)
