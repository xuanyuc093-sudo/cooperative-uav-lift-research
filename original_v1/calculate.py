"""Reproduce analytical screening calculations; this is NOT a flight simulator.
Run: python calculate.py. Requires numpy and matplotlib.
"""
from pathlib import Path
import csv
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
(ROOT / 'figures').mkdir(exist_ok=True)
G = 9.81
N = 3
MI = 0.6
T_REF = 20.0
KAPPA = 0.8
ETA = 0.8
T_AVAILABLE = KAPPA * T_REF
T_OPERATIONAL = ETA * T_AVAILABLE
THETA = math.radians(20)

def hover(ml, theta=THETA):
    share = ml / N
    tension = share * G / math.cos(theta)
    vertical = (MI + share) * G
    horizontal = share * G * math.tan(theta)
    f = math.hypot(vertical, horizontal)
    return dict(payload_kg=ml, total_mass_kg=N*MI+ml,
                lambda_available=N*T_AVAILABLE/((N*MI+ml)*G),
                lambda_operational=N*T_OPERATIONAL/((N*MI+ml)*G),
                tension_per_cable_N=tension, thrust_per_uav_N=f,
                thrust_fraction_of_available=f/T_AVAILABLE,
                thrust_fraction_of_operational=f/T_OPERATIONAL,
                vehicle_tilt_deg=math.degrees(math.atan2(horizontal, vertical)))

def skew(r):
    x,y,z=r
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])

phis=np.arange(N)*2*math.pi/N
attach=np.array([[.15*math.cos(p),.15*math.sin(p),.05] for p in phis])
u=np.array([[math.sin(THETA)*math.cos(p),math.sin(THETA)*math.sin(p),math.cos(THETA)] for p in phis])
Gmat=np.vstack([np.tile(np.eye(3),(1,N)),np.hstack([skew(r) for r in attach])])
Amat=np.column_stack([np.r_[ui,np.cross(r,ui)] for r,ui in zip(attach,u)])
rows=[hover(x) for x in [.3,.6,1.,1.4,1.8]]
baseline=rows[1]
payload_pos=np.array([0.,0.,1.])
uav_pos=payload_pos+attach+u
tension=baseline['tension_per_cable_N']
resultant=(u*tension).sum(axis=0)
moment=np.cross(attach,u*tension).sum(axis=0)
assert np.allclose(resultant,[0,0,.6*G],atol=1e-12)
assert np.linalg.norm(moment)<1e-12
assert np.linalg.matrix_rank(Gmat)==6
assert np.linalg.matrix_rank(Amat)==3
assert np.allclose(np.linalg.norm(uav_pos-(payload_pos+attach),axis=1),1.)

distances=[float(np.linalg.norm(uav_pos[i]-uav_pos[j])) for i in range(N) for j in range(i)]
rho=1.225
cdA=.15
mass=2.4
a_brake=.5
latency=.15
distance=3.
v_brake=-a_brake*latency+math.sqrt((a_brake*latency)**2+2*a_brake*distance)
a_phi=G*math.tan(math.radians(10))
v_turn=math.sqrt(a_phi*2)
f_horizontal=min(math.sqrt((N*T_OPERATIONAL)**2-(mass*G)**2),mass*G*math.tan(math.radians(25)))
v_drag=math.sqrt(2*f_horizontal/(rho*cdA))
pendulum=[dict(length_m=l, fixed_support_period_s=2*math.pi*math.sqrt(l/G)) for l in [.5,1,2,5,20]]
summary=dict(gravity=G,hover=rows,baseline_uav_positions_m=uav_pos.tolist(),
             baseline_thrust_vectors_N=(MI*G*np.array([0,0,1])+tension*u).tolist(),
             baseline_inter_uav_distances_m=distances,baseline_wrench_residual_N_Nm=[*resultant-[0,0,.6*G],*moment],
             vector_force_map_rank=int(np.linalg.matrix_rank(Gmat)),fixed_direction_tension_map_rank=int(np.linalg.matrix_rank(Amat)),
             screening=dict(braking_speed_m_s=v_brake,turning_speed_at_radius_2m_m_s=v_turn,
                            equivalent_pendulum_acceleration_limit_m_s2=a_phi,
                            aggregate_drag_speed_no_wind_no_acceleration_m_s=v_drag),
             pendulum_reference=pendulum)
(ROOT/'calculations.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
with (ROOT/'hover_sweep.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)

params={
 'scope':'Hypothetical baseline B0. Analytical consistency verified. No closed-loop validation or flight authorization.',
 'units':'SI; world z upward; gravity [0,0,-9.81]',
 'n_uav':3,'uav_mass_kg':.6,'payload_mass_kg':.6,'cable_length_m':1.,
 'uav_thrust_reference_N':20.,'physical_derating_assumption':.8,'operational_fraction_of_available':.8,
 'uav_thrust_available_N':16.,'uav_thrust_operational_N':12.8,
 'payload_box_dimensions_m':[.3,.3,.1],
 'payload_inertia_kg_m2':[.005,.005,.009],
 'uav_inertia_assumption_kg_m2':[.003,.003,.005],
 'payload_attachments_body_m':attach.tolist(),'uav_attachment_body_m':[0,0,0],
 'initial_cable_inclination_deg':20.,'initial_payload_position_m':payload_pos.tolist(),
 'initial_uav_positions_m':uav_pos.tolist(),
 'initial_uav_thrust_axis_world':[(f/np.linalg.norm(f)).tolist() for f in MI*G*np.array([0,0,1])+tension*u],
 'initial_velocities':'zero','initial_angular_velocities':'zero','initial_payload_rotation':'identity',
 'initial_uav_yaw_rad':0.,'motor_first_order_time_constant_s_assumption':.04,
 'cable_mass_base_model_kg':0.,'cable_tension_only':True,
 'cable_structural_limit_N_assumption':10.,'planning_tension_min_N_assumption':.3,
 'max_vehicle_tilt_deg_assumption':25.,'nominal_speed_m_s_assumption':1.,
 'nominal_acceleration_m_s2_assumption':.5,'nominal_jerk_m_s3_assumption':1.,
 'equivalent_dynamic_swing_warning_deg_assumption':10.,
 'wind_base_m_s':[0,0,0],'wind_sensitivity_m_s':[0,1,3,5],
 'rho_kg_m3':1.225,'uav_cdA_m2_assumption':[.02,.02,.01],
 'payload_cdA_m2_assumption':[.09,.09,.09],
 'delay_nominal_s_assumption':.02,'delay_sensitivity_s':[0,.02,.05,.10],
 'simulation_step_s_assumption':.001,'attitude_update_hz_assumption':500,
 'geometric_outer_update_hz_assumption':100,'nmpc_update_hz_initial_assumption':20,
 'provenance':{
 'uav_mass_kg':'Sun et al. 2025 experimental scale; assumed adopted for B0',
 'payload_mass_kg':'Research design choice; not a published experiment result',
 'cable_length_m':'Sun et al. 2025 experimental cable length adopted',
 'uav_thrust_reference_N':'Sun et al. 2025 Fig.2 thrust-limit experiment reference; adopted control cap, not certified continuous thrust',
 'payload_inertia_kg_m2':'Calculated uniform box inertia m/12 * [b^2+c^2,a^2+c^2,a^2+b^2]',
 'other_parameters':'Explicit research assumptions pending identification; see report parameter table'
 }}
(ROOT/'baseline_B0.json').write_text(json.dumps(params,ensure_ascii=False,indent=2))

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
mls=np.linspace(0,2.,201)
fig,axs=plt.subplots(1,2,figsize=(10,3.7),layout='constrained')
axs[0].plot(mls,[hover(m)['lambda_available'] for m in mls],label='Available thrust / total weight')
axs[0].plot(mls,[hover(m)['lambda_operational'] for m in mls],label='Operational thrust / total weight')
axs[0].axhline(1,color='gray',ls='--');axs[0].set(xlabel='Payload mass (kg)',ylabel='Thrust-to-weight ratio',title='3 UAVs, 0.6 kg each; 16 N available each');axs[0].legend(fontsize=8)
for angle in [0,20,45,60]:
 axs[1].plot(mls,[hover(m,math.radians(angle))['thrust_per_uav_N'] for m in mls],label=f'Cable inclination {angle} deg')
axs[1].axhline(T_OPERATIONAL,color='black',ls='--',label='Operational cap 12.8 N')
axs[1].set(xlabel='Payload mass (kg)',ylabel='Required hover thrust per UAV (N)',title='Symmetric hover: cable spread consumes thrust');axs[1].legend(fontsize=8)
fig.savefig(ROOT/'figures/01_payload_thrust.png',dpi=180);plt.close(fig)

fig,axs=plt.subplots(1,2,figsize=(10,3.7),layout='constrained')
vs=np.linspace(0,6,200)
for ab in [.5,1.,2.]:axs[0].plot(vs,vs*.15+vs**2/(2*ab),label=f'Deceleration {ab:g} m/s^2')
axs[0].set(xlabel='Ground speed (m/s)',ylabel='Delay + braking distance (m)',title='Idealized constant-deceleration screening');axs[0].legend(fontsize=8)
rs=np.linspace(.2,10,200)
for ap in [.5,1.,a_phi]:axs[1].plot(rs,np.sqrt(ap*rs),label=f'Lateral acceleration {ap:.2f} m/s^2')
axs[1].set(xlabel='Turn radius (m)',ylabel='Speed bound (m/s)',title='v = sqrt(a R); dynamic verification still required');axs[1].legend(fontsize=8)
fig.savefig(ROOT/'figures/02_speed_constraints.png',dpi=180);plt.close(fig)

fig,axs=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for ax,mode in zip(axs,['Common payload attachment','Distributed payload attachments']):
 ax.plot([-.4,.4],[0,0],lw=8,color='#4e5966')
 starts=[0,0,0] if mode.startswith('Common') else [-.3,0,.3]
 ends=[-.8,0,.8]
 for x0,x1 in zip(starts,ends):
  ax.plot([x0,x1],[.03,1.],color='#d18a36',lw=2)
  ax.plot([x1-.13,x1+.13],[1.,1.],color='#276e9c',lw=4)
  ax.scatter(x0,.03,color='#d18a36',s=30,zorder=3)
 ax.scatter(0,-.12,color='black',s=20);ax.text(.04,-.17,'CoM',fontsize=9)
 ax.set(title=mode,xlim=(-1.1,1.1),ylim=(-.35,1.3),aspect='equal');ax.axis('off')
fig.suptitle('Schematic side view; three non-collinear points require a 3D layout',fontsize=11)
fig.savefig(ROOT/'figures/03_attachment_topologies.png',dpi=180);plt.close(fig)
print(json.dumps(summary,ensure_ascii=False,indent=2))
