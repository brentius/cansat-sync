"""GPL-3.0-only. Independent checks for equations not exercised by old suite."""
import math
import unittest
import numpy as np
from models import *
from feasibility import *


class FeasibilityChecks(unittest.TestCase):
    def test_ncc_integrals_against_direct_all_windows(self):
        rng = np.random.default_rng(19)
        for shape, template_shape in [((19,23),(4,7)),((17,20),(8,5))]:
            image = rng.normal(size=shape); template = rng.normal(size=template_shape)
            got = ncc_fft(image, template); expected = np.empty_like(got)
            t = template-template.mean()
            for y,x in np.ndindex(got.shape):
                w = image[y:y+len(t), x:x+t.shape[1]]; w = w-w.mean()
                expected[y,x] = np.sum(w*t)/math.sqrt(np.sum(w*w)*np.sum(t*t))
            np.testing.assert_allclose(got, expected, atol=2e-14, rtol=0)
            np.testing.assert_allclose(ncc_fft(image*3+1e5,template), got, atol=2e-11, rtol=0)
        self.assertTrue(np.all(np.isnan(ncc_fft(np.ones((20,20)),rng.normal(size=(3,4))))))
        for im,t in [(np.ones((3,3)),np.ones((4,4))),(np.array([[np.nan]]),np.ones((1,1)))]:
            with self.assertRaises(ValueError): ncc_fft(im,t)

    def test_coarse_ncc_recovers_large_shift(self):
        rng = np.random.default_rng(25); image = rng.normal(size=(364,364))
        template = image[211:275, 87:151].copy()
        got = ncc_fft(image,template)
        self.assertEqual(np.unravel_index(np.nanargmax(got),got.shape),(211,87))
        self.assertAlmostEqual(got[211,87],1.,places=11)

    def test_eskf_dynamics_nonlinear_finite_difference(self):
        r = rotation(np.array([.1,-.2,.3])); force = np.array([.3,-.5,9.8]); omega = np.array([.7,-.2,.3])
        f,_ = eskf_matrices(r,force,omega)
        eps = 1e-6
        def error_derivative(error):
            dv = r@rotation(error[6:9])@(force-error[9:12])-r@force
            # Exact right-coordinate instantaneous attitude error derivative.
            dtheta = np.linalg.solve(right_jacobian(error[6:9]),
                                    omega-error[12:15]-rotation(-error[6:9])@omega)
            return np.r_[error[3:6],dv,dtheta,np.zeros(6)]
        fd = np.column_stack([(error_derivative(eps*a)-error_derivative(-eps*a))/(2*eps) for a in np.eye(15)])
        np.testing.assert_allclose(fd, f, atol=2e-9, rtol=0)

    def test_15state_discretization_covariance_ode(self):
        f,d = eskf_matrices(rotation([.1,.2,-.1]),np.array([.4,.2,9.8]),np.array([.3,-.4,.5]))
        phi,q = discretize(f,d,.01)
        state = np.r_[np.eye(15).ravel(),np.zeros(225)]
        def fun(t,x):
            a,b = x[:225].reshape(15,15),x[225:].reshape(15,15)
            return np.r_[(f@a).ravel(),(f@b+b@f.T+d).ravel()]
        got = rk4(fun,state,.0001,100)
        np.testing.assert_allclose(phi,got[:225].reshape(15,15),atol=2e-14,rtol=0)
        np.testing.assert_allclose(q,got[225:].reshape(15,15),atol=1e-15,rtol=0)
        self.assertGreaterEqual(np.linalg.eigvalsh(q).min(),-1e-15)

    def test_joint_correlated_update_against_batch_gaussian(self):
        rng=np.random.default_rng(5); l=rng.normal(size=(8,11)); joint=l@l.T
        p,c,v=joint[:5,:5],joint[:5,5:],joint[5:,5:]
        h=rng.normal(size=(3,5)); y=rng.normal(size=(3,4))
        got,post,s=gaussian_update(np.zeros((5,4)),p,y,h,v,c)
        transform=np.block([[np.eye(5),np.zeros((5,3))],[h,np.eye(3)]])
        combined=transform@joint@transform.T
        gain=np.linalg.solve(combined[5:,5:],combined[5:,:5]).T
        np.testing.assert_allclose(got,gain@y,atol=1e-14)
        np.testing.assert_allclose(post,combined[:5,:5]-gain@combined[5:,:5],atol=1e-14)

    def test_pose_fit_from_independent_noiseless_projection(self):
        true=np.array([3.,-2.,150.]); r=np.diag([1.,-1.,-1.])@rotation([.07,-.1,.2])
        xy=np.array([(x,y) for x in (-75,0,75) for y in (-45,0,45)])
        points=np.column_stack((xy,np.zeros(len(xy))))
        observed,_=camera_pixels(points,true,r)
        fitted,attitude,p=fit_pose(points,observed,true+np.array([2.,-1.,.5]),r@rotation([.01,-.02,.01]))
        np.testing.assert_allclose(fitted,true,atol=1e-10)
        np.testing.assert_allclose(attitude,r,atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(p).min(),0)

    def test_renderer_ray_plane_and_interpolation(self):
        pos=np.array([3.,-2.,100.]); r=np.diag([1.,-1.,-1.])
        image=render_image(pos,r,size=(40,64),focal=32.)
        yy,xx=np.indices(image.shape)
        np.testing.assert_allclose(image,terrain(3+100*(xx-32)/32,-2-100*(yy-20)/32),atol=1e-12)
        self.assertTrue(np.isnan(bilinear(image,np.array([[-1.,2.]])))[0])

    def test_joint_quadratic_peak(self):
        yy,xx=np.mgrid[:9,:9]; delta=np.stack((xx-4.23,yy-3.72),axis=-1)
        score=1-np.einsum('...i,ij,...j->...',delta,np.array([[3.,1.],[1.,2.]]),delta)
        np.testing.assert_allclose(quadratic_peak(score),[4.23,3.72],atol=1e-13)

    def test_photometric_information_schur(self):
        rng=np.random.default_rng(8); texture=rng.normal(size=100); d=rng.normal(size=(100,2))
        b=np.column_stack((texture,np.ones(100))); gain=1.4; sd=2.
        design=np.column_stack((gain*d,b)); info=design.T@design/sd**2
        marginal=info[:2,:2]-info[:2,2:]@np.linalg.solve(info[2:,2:],info[2:,:2])
        projection=np.eye(100)-b@np.linalg.solve(b.T@b,b.T)
        np.testing.assert_allclose(marginal,gain**2/sd**2*d.T@projection@d,atol=3e-14)
        np.testing.assert_allclose(projection@b,0.,atol=2e-15)

    def test_optical_flow_against_moving_point_projection(self):
        pc=np.array([20.,-15.,90.]); v=np.array([5.,1.,10.]); omega=np.array([.4,-.7,2.])
        x,y=pc[:2]/pc[2]; z=pc[2]
        predicted=np.array([(-v[0]+x*v[2])/z+x*y*omega[0]-(1+x*x)*omega[1]+y*omega[2],
                            (-v[1]+y*v[2])/z+(1+y*y)*omega[0]-x*y*omega[1]-x*omega[2]])
        eps=1e-6
        def value(t):
            p=rotation(-omega*t)@(pc-v*t)
            return p[:2]/p[2]
        np.testing.assert_allclose((value(eps)-value(-eps))/(2*eps),predicted,atol=3e-11)

    def test_density_terminal_pressure_and_richardson_jacobians(self):
        def finite(fun,x):
            return np.array([(fun(x+1e-5*a)-fun(x-1e-5*a))/(2e-5) for a in np.eye(len(x))])
        # Terminal speed including drag area, density equation and height mean.
        x=np.array([.325,1.2,1.3,.037]); vt=math.sqrt(2*x[0]*G0/np.prod(x[1:]))
        np.testing.assert_allclose(finite(lambda a:math.sqrt(2*a[0]*G0/np.prod(a[1:])),x),
                                   vt/2*np.array([1/x[0],-1/x[1],-1/x[2],-1/x[3]]),rtol=1e-7)
        p,t=99000.,288.; rho=p/(287.05*t)
        np.testing.assert_allclose(finite(lambda a:a[0]/(287.05*a[1]),np.array([p,t])),[rho/p,-rho/t],rtol=2e-6)
        x=np.array([.1,20.,1.,.4,290.]); q,h,a,b,theta=x; den=a*a+b*b
        ri=G0/theta*q*h/den
        jac=np.array([G0/theta*h/den,G0/theta*q/den,-2*ri*a/den,-2*ri*b/den,-ri/theta])
        np.testing.assert_allclose(finite(lambda x:G0/x[4]*x[0]*x[1]/(x[2]**2+x[3]**2),x),jac,rtol=1e-7)

    def test_thermistor_ramp_and_inverse(self):
        tau,slope=.7,.008; initial=285.; duration=10.
        def air(t): return initial+slope*10.5*t
        got=rk4(lambda t,x:(air(t)-x)/tau,[initial],.001,10000)[0]
        exact=air(duration)-tau*slope*10.5*(1-math.exp(-duration/tau))
        self.assertAlmostEqual(got,exact,places=10)
        derivative=(air(duration)-got)/tau
        self.assertAlmostEqual(got+tau*derivative,air(duration),places=12)

    def test_forced_pendulum_frequency_response(self):
        w0=math.sqrt(G0/1.5); damping=.15; omega=2.; force=.3
        h=1/(w0*w0-omega*omega+2j*damping*w0*omega)
        # Start at exact steady state; numerical ODE is independent of transfer formula.
        initial=[force*h.real,force*(1j*omega*h).real]
        fun=lambda t,x:np.array([x[1],force*math.cos(omega*t)-2*damping*w0*x[1]-w0*w0*x[0]])
        got=rk4(fun,initial,.001,10000)
        exact=[force*(h*np.exp(1j*omega*10)).real,force*(1j*omega*h*np.exp(1j*omega*10)).real]
        np.testing.assert_allclose(got,exact,atol=3e-13)

    def test_incompressible_tensor_divergence(self):
        r=np.array([.7,-.2,1.1]); length=2.; eps=1e-5
        def tensor(r):
            distance=np.linalg.norm(r); unit=r/distance
            longitudinal=math.exp(-distance/length); transverse_value=(1-distance/(2*length))*longitudinal
            return transverse_value*np.eye(3)+(longitudinal-transverse_value)*np.outer(unit,unit)
        divergence=sum((tensor(r+eps*a)[:,j]-tensor(r-eps*a)[:,j])/(2*eps) for j,a in enumerate(np.eye(3)))
        np.testing.assert_allclose(divergence,0.,atol=2e-11)

    def test_transfer_function_resolvent_not_comma(self):
        for omega in (0., .1, 1., 3., 10.):
            direct=(C@np.linalg.solve(1j*omega*np.eye(2)-F,B)).item()
            formula=(1+math.sqrt(3)*1j*omega)/(1+1j*omega)**2
            self.assertLess(abs(direct-formula),1e-15)
            self.assertAlmostEqual(abs(formula)**2,(1+3*omega**2)/(1+omega**2)**2,places=14)

    def test_taylor_goldstein_transformation(self):
        # Identity for arbitrary trial function, not a fabricated unstable eigenmode.
        z=.4; c=.3+.2j; n2=1.; k=.8
        u=lambda z:1+.5*z+.2*z*z
        trial=lambda z:np.sin(1.3*z)+.2j*np.cos(.8*z)
        phi=lambda z:np.sqrt(u(z)-c)*trial(z)
        eps=2e-4; v=u(z)-c; up=.5+.4*z; upp=.4
        fp=1.3*np.cos(1.3*z)-.16j*np.sin(.8*z)
        fpp=-1.3**2*np.sin(1.3*z)-.128j*np.cos(.8*z)
        original=(phi(z+eps)-2*phi(z)+phi(z-eps))/eps**2+(n2/v**2-upp/v-k*k)*phi(z)
        transformed=(v*fpp+up*fp+((n2-up*up/4)/v-upp/2-k*k*v)*trial(z))/np.sqrt(v)
        self.assertLess(abs(original-transformed),2e-8)

    def test_neutral_gls_and_roughness_gradient(self):
        z=np.linspace(10,80,30); x=np.column_stack((np.log(z),np.ones(len(z))))
        truth=np.array([.75,-.75*math.log(.1)]); covariance=.1**2*.6**abs(np.subtract.outer(np.arange(30),np.arange(30)))
        info=x.T@np.linalg.solve(covariance,x)
        gain=np.linalg.solve(info,x.T@np.linalg.solve(covariance,np.eye(30)))
        np.testing.assert_allclose(gain@(x@truth),truth,atol=1e-13)
        self.assertAlmostEqual(math.exp(-(gain@(x@truth))[1]/(gain@(x@truth))[0]),.1,places=13)
        jac=np.array([truth[1]/truth[0]**2,-1/truth[0]])
        eps=1e-6
        fd=np.array([((-(truth+eps*a)[1]/(truth+eps*a)[0])-(-(truth-eps*a)[1]/(truth-eps*a)[0]))/(2*eps) for a in np.eye(2)])
        np.testing.assert_allclose(fd,jac,atol=5e-10)

    def test_layer_overlap_integral_and_covariance(self):
        edges=np.arange(0.,302.4,2.1); weights=overlap_weights(edges,80,100)
        self.assertAlmostEqual(weights.sum(),1.,places=14)
        self.assertAlmostEqual(weights@((edges[:-1]+edges[1:])/2),90.,delta=.03)
        # Independent quadrature for constant-scale layer covariance.
        z=np.linspace(0,20,801); quadrature=np.ones(len(z)); quadrature[[0,-1]]=.5; quadrature/=800
        cov=transverse(np.subtract.outer(z,z)/11.)
        numerical=quadrature@cov@quadrature
        r=np.linspace(0,20,20001)
        reduced=2*np.trapezoid((20-r)*transverse(r/11.),r)/20**2
        self.assertAlmostEqual(numerical,reduced,delta=3e-6)

    def test_time_height_speed_covariance(self):
        h,vd=300.,10.5; cov=np.array([[.25,.06],[.06,.09]])
        jac=np.array([1/vd,-h/vd**2])
        formula=cov[0,0]/vd**2+h*h*cov[1,1]/vd**4-2*h*cov[0,1]/vd**3
        self.assertAlmostEqual(jac@cov@jac,formula,places=14)
        z=np.linspace(0,h,30001); scale_height=8400.
        numeric=np.trapezoid(np.exp(-z/(2*scale_height))/vd,z)
        exact=2*scale_height/vd*(1-math.exp(-h/(2*scale_height)))
        self.assertAlmostEqual(numeric,exact,places=10)

    def test_expected_hazard_cost_and_control_bound(self):
        # Straight half-plane hazard: convolution reduces to a Gaussian CDF.
        mean,sd=-8.,3.; rng=np.random.default_rng(13)
        rate=np.mean(mean+sd*rng.normal(size=200000)>=0)
        exact=.5*math.erfc(-mean/(sd*math.sqrt(2)))
        self.assertLess(abs(rate-exact),5*math.sqrt(exact*(1-exact)/200000))
        kappa=.3; height=100.; t=np.linspace(0,height/10.5,10001)
        displacement=np.trapezoid(np.full(len(t),kappa*10.5),t)
        self.assertAlmostEqual(displacement,kappa*height,places=12)

    def test_tls_lift_and_analytic_dual_certificate(self):
        points=np.array([[0.,0.],[1,0],[0,1],[1,1.]])
        q=np.array([1.2,.3,2.,-1.]); targets=np.array([[x*q[0]-y*q[1]+q[2],x*q[1]+y*q[0]+q[3]] for x,y in points])
        objective,slack,lower=tls_certificate(points,targets,np.ones(4))
        candidate=lift_tls(q,np.ones(4)); upper=candidate@objective@candidate
        self.assertLess(abs(upper-lower),1e-13)
        self.assertGreaterEqual(np.linalg.eigvalsh(slack).min(),-1e-13)
        np.testing.assert_allclose(slack@candidate,0.,atol=1e-14)
        for switches in [np.zeros(4),np.array([1.,0.,1.,0.])]:
            x=lift_tls(q,switches)
            self.assertAlmostEqual(x@objective@x,4-switches.sum(),places=13)

    def test_queue_capacity_boundary_and_storage(self):
        wait,latency=queue_simulation(np.full(150,.25))
        self.assertAlmostEqual(wait[-1],7.45,places=10)
        self.assertAlmostEqual(latency[-1],7.7,places=10)
        wait,latency=queue_simulation(np.full(150,.25),workers=2)
        np.testing.assert_allclose(wait,0.,atol=1e-13)
        self.assertAlmostEqual(5120000*.5,2560000)

    def test_energy_load_radio_and_binomial_limits(self):
        # Illustrative bench inputs only; energy conservation and engineering units.
        powers=np.array([2.,5.]); hours=np.array([3.,30/3600]); eta=np.array([.85,.85])
        required=np.sum(powers*hours/eta); capacity=required/(3.7*.8)
        self.assertAlmostEqual(capacity*3.7*.8,required)
        self.assertAlmostEqual(.015*20*G0,2.941995,places=10)
        distance,frequency=1000.,868e6; c=299792458.
        loss=20*math.log10(4*math.pi*distance*frequency/c)
        self.assertAlmostEqual(loss,91.21863097285026,delta=.001)
        m=2000; bound=1-.05**(1/m)
        self.assertAlmostEqual((1-bound)**m,.05,places=13)

    def test_lever_arm_and_pendulum_specific_force(self):
        r=rotation([.1,-.2,.3]); lever=np.array([.1,.2,-.05]); omega=np.array([.4,-.3,.6]); eps=1e-6
        fd=(r@rotation(eps*omega)@lever-r@rotation(-eps*omega)@lever)/(2*eps)
        np.testing.assert_allclose(fd,r@np.cross(omega,lever),atol=3e-11)
        angle=.25; length=1.5; rate=.7
        e=np.array([math.sin(angle),0.,-math.cos(angle)])
        # Fixed pivot, tension/m=g cos(theta)+L theta_dot^2.
        tension=G0*math.cos(angle)+length*rate**2
        gravity=np.array([0.,0.,-G0]); acceleration=gravity-tension*e
        body=np.column_stack(([math.cos(angle),0.,math.sin(angle)],[0.,1.,0.],e))
        specific=body.T@(acceleration-gravity)
        np.testing.assert_allclose(specific,[0.,0.,-tension],atol=2e-15)
        self.assertGreater(abs(angle),.2)

    def test_stable_most_gradient_relation_and_ill_conditioning(self):
        for zeta in (.001,.1,1.,10.):
            phi=1+5*zeta; ri=zeta*phi/phi**2
            self.assertAlmostEqual(ri,zeta/(1+5*zeta),places=15)
            self.assertAlmostEqual(ri/(1-5*ri),zeta,places=12)
        self.assertGreater(1/(1-5*.199)**2,39000)

    def test_homography_determinant_and_datum_gauge(self):
        r=np.diag([1.,-1.,-1.])@rotation([.1,-.2,.3]); pos=np.array([3.,-2.,150.]); k=np.diag([320.,320.,1.])
        rcw=r.T; h=k@np.column_stack((rcw[:,0],rcw[:,1],-rcw@pos))
        self.assertAlmostEqual(np.linalg.det(h)/np.linalg.det(k),-pos[2],places=12)
        points=np.array([[20.,30.,0.],[-40.,10.,0.]])
        pix,_=camera_pixels(points,pos,r)
        q=rotation([.05,-.1,.3]); shift=np.array([10.,-5.,2.])
        moved,_=camera_pixels(points@q.T+shift,q@pos+shift,q@r)
        np.testing.assert_allclose(pix,moved,atol=1e-13)

    def test_truth_difference_and_landing_cross_covariance(self):
        rng=np.random.default_rng(21); a=rng.normal(size=(4,6)); joint=a@a.T
        sum_map=np.column_stack((np.eye(2),np.eye(2))); difference=np.column_stack((np.eye(2),-np.eye(2)))
        p1,p2,cross=joint[:2,:2],joint[2:,2:],joint[:2,2:]
        np.testing.assert_allclose(sum_map@joint@sum_map.T,p1+p2+cross+cross.T,atol=3e-15)
        np.testing.assert_allclose(difference@joint@difference.T,p1+p2-cross-cross.T,atol=3e-15)

    def test_spectral_variance_and_brownian_time_change(self):
        omega=np.linspace(0,100,100001); spectrum=(1+3*omega**2)/(1+omega**2)**2
        finite=np.trapezoid(spectrum,omega)/math.pi
        tail=1-(2*math.atan(100)-100/(1+100**2))/math.pi
        self.assertAlmostEqual(finite+tail,1.,places=10)
        length,vd,dt=35.,10.5,.2
        phi_s,q_s=discretize(F/length,B@B.T/length,vd*dt)
        phi_t,q_t=discretize(F*vd/length,B@B.T*vd/length,dt)
        np.testing.assert_allclose(phi_s,phi_t,atol=1e-15)
        np.testing.assert_allclose(q_s,q_t,atol=1e-15)

    def test_warp_integrals_and_intensity_units(self):
        z=np.linspace(10,100,20001); a=.5; linf=100.
        numerical=np.trapezoid(1/(a*z/(1+a*z/linf)),z)
        exact=math.log(z[-1]/z[0])/a+(z[-1]-z[0])/linf
        self.assertAlmostEqual(numerical,exact,delta=4e-8)
        zmetres=6.096; height_feet=zmetres/.3048; wind20=5.
        self.assertAlmostEqual(height_feet,20.,places=14)
        sigma_w=.1*wind20; sigma_h=sigma_w/(.177+.000823*height_feet)**.4
        self.assertGreater(sigma_h,sigma_w)

    def test_frequency_lag_response_and_averaging_kernel_eigenvalues(self):
        ell=10.5**2/G0; k=2*math.pi/20
        self.assertAlmostEqual(abs(1+1j*k*ell),math.sqrt(1+(k*ell)**2))
        # A=GK is nonsymmetric; prior whitening gives symmetric bounded spectrum.
        z=np.linspace(100,2,30); sa=wind_cov(z); forward=lag_matrix(30,3.,ell); se=.36*np.eye(30)
        gain,ak,post=retrieval(forward,sa,se)
        vals=np.linalg.eigvals(ak)
        self.assertLess(abs(vals.imag).max(),1e-12)
        self.assertGreater(vals.real.min(),-1e-12); self.assertLess(vals.real.max(),1+1e-12)
        self.assertAlmostEqual(np.trace(ak),vals.real.sum(),places=12)
        eps=1e-5; direction=np.linspace(-1,1,30)
        np.testing.assert_allclose((gain@(forward*(1+eps))@direction-gain@forward@direction)/eps,
                                   ak@direction,atol=1e-10)


if __name__=='__main__': unittest.main(verbosity=2)
