"""Independent 03:00 checkpoint audit; does NOT alter the running transport model.

The X-quad dimensions, per-rotor limits, drag-torque ratio and slew limit are
explicit hypothetical inputs. Impacts are ideal rigid, perfectly inelastic events.
"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib
import json
import csv
import math
import collections
import numpy as np
from scipy.optimize import linprog,lsq_linear
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
plt.rcParams.update({'font.family':['Noto Sans CJK JP','DejaVu Sans'],'font.size':10})

def skew(r):
    x,y,z=r
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])

def allocate():
    arm=.17;a=arm/math.sqrt(2);yaw_ratio=.016;limit=4.
    positions=np.array([[a,a,0],[-a,a,0],[-a,-a,0],[a,-a,0]])
    B=np.vstack([np.ones(4),positions[:,1],-positions[:,0],yaw_ratio*np.array([1,-1,1,-1])])
    desired=np.array([12.8,.1,0.,.05])
    exact=np.linalg.solve(B,desired)
    assert np.max(np.abs(B@exact-desired))<1e-12
    assert sum(exact)<4*limit and max(exact)>limit
    clipped=np.clip(exact,0,limit)
    # Dimensionless residuals. WLS is an illustrative allocation, not a replication
    # of the INDI paper and not a proof of closed-loop stability.
    normalizer=np.diag(1/np.array([16.,a*8,a*8,yaw_ratio*8]))
    W=np.diag([10.,100.,100.,1.])
    solution=lsq_linear(W@normalizer@B,W@normalizer@desired,bounds=(0,limit),tol=1e-13,lsmr_tol=1e-13)
    assert solution.success
    optimized=solution.x
    rows=[]
    for roll in [0.,.05,.1,.2,.3]:
        for yaw in [0.,.02,.04,.05,.06]:
            tau=np.array([roll,0.,yaw])
            lp=linprog(-np.ones(4),A_eq=B[1:],b_eq=tau,bounds=[(0,limit)]*4,method='highs')
            rows.append(dict(roll_Nm=roll,yaw_Nm=yaw,max_collective_N=float(-lp.fun) if lp.success else None,
                             hypothetical_per_rotor_cap_N=limit))
    with (ROOT/'results'/'allocation_collective_envelope.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    F=np.linspace(0,16,401)
    yawcap=yaw_ratio*np.minimum(F,16-F)
    rolls=np.linspace(0,.3,151);yaws=np.linspace(0,.10,151)
    feasible=np.zeros((len(yaws),len(rolls)))
    for j,ty in enumerate(yaws):
        for i,tx in enumerate(rolls):
            rotor=np.linalg.solve(B,[12.8,tx,0.,ty])
            feasible[j,i]=float(np.min(rotor)>=-1e-10 and np.max(rotor)<=limit+1e-10)
    fig,ax=plt.subplots(1,2,figsize=(10,3.7),layout='constrained')
    ax[0].plot(F,yawcap);ax[0].axvline(12.8,color='gray',ls='--')
    ax[0].set(xlabel='总推力（N）',ylabel='最大纯偏航力矩（N·m）',title='同一电机上限下：推力与姿态能力耦合')
    ax[1].pcolormesh(rolls,yaws,feasible,cmap='Blues',vmin=0,vmax=1,shading='nearest')
    ax[1].plot(.1,.05,'rx',ms=8,label='算例需求：不可行')
    ax[1].set(xlabel='滚转力矩（N·m）',ylabel='偏航力矩（N·m）',title='固定总推力12.8 N：深色为静态分配可行');ax[1].legend(fontsize=8)
    fig.savefig(ROOT/'figures'/'night_allocation_feasibility.png',dpi=180);plt.close(fig)
    # Force-rate example, stated independently from measured RPM time constants.
    previous=np.full(4,7.8804223567194756/4)
    request=np.full(4,9./4);slew=20.;control_dt=.01
    rate_upper=np.minimum(limit,previous+slew*control_dt)
    rate_lower=np.maximum(0,previous-slew*control_dt)
    rate_feasible=bool(np.all((request>=rate_lower)&(request<=rate_upper)))
    return dict(inputs=dict(arm_m=arm,x_y_coordinate_m=a,yaw_drag_ratio_m=yaw_ratio,per_rotor_force_cap_N=limit,
                            scope='Hypothetical static X-quad. No actuator identification or flight validation.'),
                allocation_matrix=B.tolist(),desired_wrench_N_Nm=desired.tolist(),
                exact_unbounded_rotor_forces_N=exact.tolist(),scalar_total_check=True,rotor_feasible=False,
                clipped=dict(rotors_N=clipped.tolist(),achieved_wrench=(B@clipped).tolist(),residual=(B@clipped-desired).tolist()),
                weighted_least_squares=dict(rotors_N=optimized.tolist(),achieved_wrench=(B@optimized).tolist(),
                    residual=(B@optimized-desired).tolist(),normalized_weights=[10,100,100,1]),
                rate_example=dict(previous_force_N=previous.tolist(),requested_force_N=request.tolist(),
                    assumed_force_slew_N_s=slew,period_s=control_dt,rate_feasible=rate_feasible,
                    minimum_constant_slew_transition_s=float(max(abs(request-previous))/slew)),
                one_rotor_failed_observation='With one rotor unavailable, zero roll/pitch/yaw torques and positive thrust generally cannot all be maintained by a conventional fixed-pitch X quad. Relaxing yaw/spinning recovery is a different dynamic task.')

def impact():
    # Generalized velocities [load translation, load body omega, UAV translations].
    mass_load=.6;mass_uav=.6;Jbody=np.array([.005,.005,.009])
    phis=np.arange(3)*2*np.pi/3;theta=math.radians(20)
    r=np.array([[.15*np.cos(p),.15*np.sin(p),.05] for p in phis])
    u=np.array([[np.sin(theta)*np.cos(p),np.sin(theta)*np.sin(p),np.cos(theta)] for p in phis])
    M=np.diag(np.r_[np.full(3,mass_load),Jbody,np.full(9,mass_uav)])
    invM=np.diag(1/np.diag(M))
    J=np.zeros((3,15))
    for i in range(3):
        J[i,:3]=-u[i]
        J[i,3:6]=-np.cross(r[i],u[i])
        J[i,6+3*i:9+3*i]=u[i]
    before=np.zeros(15);before[6:]=(.2*u).reshape(-1)
    H=J@invM@J.T
    impulses=np.linalg.solve(H,J@before)
    assert min(impulses)>0
    after=before-invM@J.T@impulses
    assert np.linalg.norm(J@after)<1e-12
    Ebefore=.5*before@M@before;Eafter=.5*after@M@after
    predicted_loss=.5*(J@before)@np.linalg.solve(H,J@before)
    assert abs(Ebefore-Eafter-predicted_loss)<1e-12
    # Position, velocity and angular momentum with R=I at the impact instant.
    xload=np.array([0.,0.,1.]);xuav=xload+r+u
    def momentum(v):
        P=mass_load*v[:3]+mass_uav*v[6:].reshape(3,3).sum(axis=0)
        L=np.cross(xload,mass_load*v[:3])+Jbody*v[3:6]+np.cross(xuav,mass_uav*v[6:].reshape(3,3)).sum(axis=0)
        return P,L
    P0,L0=momentum(before);P1,L1=momentum(after)
    assert np.linalg.norm(P0-P1)<1e-12 and np.linalg.norm(L0-L1)<1e-12
    # Elementary two-mass cross-check gives equal post-impact velocities.
    m1=m2=.6;v1=1.;v2=0.
    impulse=(v1-v2)/(1/m1+1/m2)
    va=v1-impulse/m1;vb=v2+impulse/m2
    assert abs(va-.5)<1e-12 and abs(vb-.5)<1e-12
    return dict(scope='Isolated ideal inelastic event calculation. NOT installed in the running elastic model.',
                impulse_N_s=impulses.tolist(),outward_velocity_before_m_s=(J@before).tolist(),
                radial_velocity_residual_after_m_s=(J@after).tolist(),
                energy_before_J=float(Ebefore),energy_after_J=float(Eafter),energy_loss_J=float(Ebefore-Eafter),
                linear_momentum_drift=float(np.linalg.norm(P0-P1)),angular_momentum_drift=float(np.linalg.norm(L0-L1)),
                two_mass_example=dict(mass_each_kg=.6,velocity_before_m_s=[1,0],impulse_N_s=impulse,
                                      velocity_after_m_s=[va,vb],energy_loss_J=.15),
                active_set_limitation='For simultaneous unilateral constraints, a negative impulse invalidates the assumed active set. Do not clip it and retain the same projection.')

def audit_records():
    manifest=json.loads((ROOT/'results'/'RUN_MANIFEST.json').read_text())
    actual=hashlib.sha256((ROOT/'simulation'/'coupled_model.py').read_bytes()).hexdigest()
    assert actual==manifest['model_sha256'],'running model source unexpectedly changed'
    records=[]
    for f in (ROOT/'results'/'cases').glob('*.json'):
        try:records.append(json.loads(f.read_text()))
        except json.JSONDecodeError:continue
    failures=[];duplicate_groups=collections.defaultdict(list)
    for r in records:
        if 'metrics'not in r:continue
        m=r['metrics'];c=m['config']
        if m['tension_min_N']< -1e-10:failures.append([r['case_id'],'negative tension'])
        if r.get('fine_step_agreement'):
            if not (r['research_screen_met'] and r['fine_step_screen_met'] and r['fine_step_metrics']['completed']):
                failures.append([r['case_id'],'inconsistent refinement label'])
        if c['wind']==0:
            normalized=dict(c);normalized.pop('seed',None)
            duplicate_groups[json.dumps(normalized,sort_keys=True)].append(r)
    repeats=[]
    for group in duplicate_groups.values():
        if len(group)<2:continue
        drift=max(g['metrics']['position_rmse_m'] for g in group)-min(g['metrics']['position_rmse_m'] for g in group)
        repeats.append(dict(case_ids=[g['case_id'] for g in group],rmse_spread_m=drift))
    assert not failures,failures
    return dict(snapshot_utc=datetime.now(timezone.utc).isoformat(),records_read=len(records),
                model_sha256=actual,source_unchanged=True,integrity_violations=failures,
                zero_wind_different_seed_repeat_groups=len(repeats),examples=repeats[:4],
                interpretation='The seed changes wind phase only. At zero wind, different seeds are deterministic repeat runs, not independent uncertainty trials.',
                runtime_model_modified=False)

if __name__=='__main__':
    data=dict(allocation=allocate(),inelastic_impact=impact(),record_audit=audit_records())
    (ROOT/'results'/'checkpoint_03_independent_audit.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
    print(json.dumps(data,ensure_ascii=False,indent=2))
