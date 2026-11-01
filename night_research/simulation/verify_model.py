"""Physical and numerical verification before any experiment batch is trusted."""
from pathlib import Path
import json
import math
import numpy as np
from coupled_model import Config,Model,Motion,run_case

ROOT=Path(__file__).resolve().parents[1]
results={}

# Static force and moment equilibrium for every represented topology.
eq={}
for topology in ['common','offset_common','distributed']:
    m=Model(Config(topology=topology));y=m.initial();force=y[31:40].reshape(3,3)
    residual=float(np.max(np.abs(m.rhs(0,y,force))))
    assert residual<1e-9,(topology,residual)
    eq[topology]=residual
results['static_equilibrium_max_state_derivative']=eq

# No compressive cable force; shortening a stretched cable dissipates energy.
m=Model(Config(gravity=0,drag_vehicle=0,drag_payload=0));y=m.initial()
y[13:22]=(y[:3]+m.r+.9*m.u0).reshape(-1)
assert np.max(m.cable_forces(y)[1])==0
results['slack_cables_generate_zero_force']=True

# Conservative closed system: linear/angular momentum and total mechanical energy.
c=Config(gravity=0,drag_vehicle=0,drag_payload=0,damping=0,dt=.00025)
m=Model(c);y=m.initial();y[13:16]+=.02*m.u0[0]
rng=np.random.default_rng(4);y[3:6]=rng.normal(0,.03,3);y[22:31]=rng.normal(0,.03,9)
y[10:13]=[.2,-.1,.1]
zero=np.zeros((3,3));E0=m.energy(y);p0,L0=m.momentum(y)
max_energy=max_p=max_L=0.
for k in range(8000):
    y=m.step(k*c.dt,y,zero)
    E=m.energy(y);p,L=m.momentum(y)
    max_energy=max(max_energy,abs(E-E0)/max(abs(E0),1e-12))
    max_p=max(max_p,float(np.linalg.norm(p-p0)))
    max_L=max(max_L,float(np.linalg.norm(L-L0)))
assert max_energy<1e-5,max_energy
assert max_p<1e-10,max_p
assert max_L<1e-7,max_L
results['conservative_system']={'relative_energy_drift_max':max_energy,'linear_momentum_drift_max':max_p,'angular_momentum_drift_max':max_L,'dt_s':c.dt,'duration_s':2.,'note':'Tension-only activation is nonsmooth. Coarse-step failure and refinement are retained in conservation_diagnostics.json.'}

# Dissipation check: positive cable damping cannot create mechanical energy.
m=Model(Config(gravity=0,drag_vehicle=0,drag_payload=0,damping=1.,dt=.001))
y=m.initial();y[13:16]+=.02*m.u0[0];old=m.energy(y);max_gain=0.
for k in range(1000):
    y=m.step(k*.001,y,zero);new=m.energy(y);max_gain=max(max_gain,new-old);old=new
assert max_gain<1e-7,max_gain
results['damped_system_max_one_step_energy_increase_J']=max_gain

# Input cone and magnitude limits; convex interpolation by first-order vector-force
# dynamics does not make this a test of actual attitude/rotor feasibility.
m=Model(Config());raw=rng.normal(0,100,(300,3));limited=m.cap_force(raw)
assert np.max(np.linalg.norm(limited,axis=1))<=m.c.force_cap+1e-10
assert np.all(limited[:,2]>=0)
assert np.all(np.linalg.norm(limited[:,:2],axis=1)<=limited[:,2]*math.tan(math.radians(m.c.tilt_deg))+1e-10)
results['vector_force_limits_checked']=True

# Reference continuity and exact end conditions, speed/acceleration/jerk limits.
motion_checks=[]
for speed in [.1,.5,1.,3.,5.]:
    mo=Motion(speed,.5,1.)
    times=np.linspace(0,mo.total_time,10001)
    states=np.array([mo.at(t) for t in times])
    assert np.max(states[:,1])<=speed+1e-9
    assert np.max(np.abs(states[:,2]))<=.5+1e-9
    assert abs(mo.at(mo.stop_time)[1])<1e-12
    assert abs(mo.at(mo.stop_time)[2])<1e-12
    motion_checks.append({'speed':speed,'distance':mo.distance,'duration':mo.duration})
results['jerk_limited_reference']=motion_checks

# Step convergence: SAME 50 Hz controller and command timing, finer physics steps.
convergence=[]
for dt in [.004,.002,.001]:
    cfg=Config(dt=dt,control_dt=.02,wind=0.,command_delay=0.)
    item=run_case(cfg,log_path=str(ROOT/'results'/f'verify_nominal_dt_{dt}.npz'))
    convergence.append({k:item[k] for k in ['position_rmse_m','position_peak_error_m','attitude_peak_error_deg','tension_peak_N','force_peak_N','completed']})
assert all(x['completed'] for x in convergence)
change=abs(convergence[-1]['position_rmse_m']-convergence[-2]['position_rmse_m'])
assert change<.002,change
results['step_convergence_dt_004_002_001']=convergence
results['fine_step_rmse_change_m']=change
results['step_convergence_scope']='Nominal taut-cable transport only. Severe wind+delay case is retained separately as an exploratory failure, not certified converged.'
results['scope']='Reduced model verification; not full quadrotor validation, not real-world validation.'
results['passed']=True
(ROOT/'results'/'model_verification.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
print(json.dumps(results,ensure_ascii=False,indent=2))
