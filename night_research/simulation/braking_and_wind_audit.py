"""Independent kinematic audit; no simulation of a real aircraft or cable system.

Uses explicitly hypothetical acceleration, jerk, delay, mass and drag inputs.
Reads the existing Motion implementation only for a cross-check and does not
modify the running experiment, its code or its records.
"""
from pathlib import Path
from datetime import datetime, timezone
import csv
import hashlib
import json
import math
import numpy as np
from scipy.optimize import brentq
from scipy.integrate import quad
from coupled_model import Motion
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def brake(v, a, j):
    if v < 0 or min(a, j) <= 0:
        raise ValueError('Require v>=0, a>0 and j>0')
    if v == 0:
        return dict(time_s=0., distance_m=0., peak_deceleration_m_s2=0.)
    if v >= a*a/j:
        tj = a/j
        hold = v/a-tj
    else:
        tj = math.sqrt(v/j)
        hold = 0.
    duration = 2*tj+hold
    return dict(time_s=duration, distance_m=v*duration/2,
                peak_deceleration_m_s2=j*tj)


def polynomial_crosscheck(v, a, j):
    b = brake(v, a, j)
    tj = min(a/j, math.sqrt(v/j))
    hold = max(0., v/a-a/j)
    x, vel, acc = 0., v, 0.
    integrated_distance = 0.
    for dt, jerk in [(tj, -j), (hold, 0.), (tj, j)]:
        integrated_distance += quad(lambda t: vel+acc*t+jerk*t*t/2, 0., dt)[0]
        x += vel*dt+acc*dt*dt/2+jerk*dt**3/6
        vel += acc*dt+jerk*dt*dt/2
        acc += jerk*dt
    assert abs(x-b['distance_m']) < 1e-9
    assert abs(integrated_distance-x) < 1e-9
    assert abs(vel) < 1e-11 and abs(acc) < 1e-11
    return max(abs(x-b['distance_m']), abs(integrated_distance-x))


def write_csv(name, rows):
    with (ROOT/'results'/name).open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def main():
    a, j, delay, reserve = .5, 1., .1, .5
    max_error = 0.
    for v in [1e-6, .1, .249999, .25, .250001, .5, 1., 2., 3., 5., 8.]:
        max_error = max(max_error, polynomial_crosscheck(v, a, j))
    rows = []
    for v in [.1, .25, .5, 1., 2., 3., 5.]:
        b = brake(v, a, j)
        motion = Motion(v, a, j)
        # Existing rest-to-rest Motion uses the same acceleration/deceleration.
        tc = motion.durations[3]
        inferred_stop_distance = (motion.distance-v*tc)/2
        assert abs(inferred_stop_distance-b['distance_m']) < 1e-9
        rows.append(dict(speed_ground_m_s=v, deceleration_limit_m_s2=a,
                         jerk_limit_m_s3=j, delay_s=delay,
                         instantaneous_acceleration_formula_m=v*delay+v*v/(2*a),
                         jerk_limited_stop_time_s=b['time_s'],
                         jerk_limited_distance_with_delay_m=v*delay+b['distance_m'],
                         hypothetical_budget_with_reserve_m=v*delay+b['distance_m']+reserve,
                         peak_deceleration_m_s2=b['peak_deceleration_m_s2']))
    write_csv('jerk_limited_braking.csv', rows)
    budgets = []
    for space in [3., 10., 30.]:
        v = brentq(lambda v: delay*v+brake(v,a,j)['distance_m']+reserve-space, 0., 30.)
        budgets.append(dict(available_distance_m=space, hypothetical_kinematic_speed_m_s=v,
                            scope='Kinematic calculation only; no closed-loop stopping guarantee'))
    # Equal co-moving airspeeds give summed drag using aggregate Cd*A=.15 m^2.
    rho, cda, mass, g = 1.225, .15, 2.4, 9.81
    vg = np.array([3., 0., 0.])
    wind_rows = []
    for label, wind in [('calm',[0.,0.,0.]),('tailwind',[3.,0.,0.]),
                        ('headwind',[-3.,0.,0.]),('crosswind',[0.,3.,0.])]:
        va = vg-np.array(wind)
        drag = -.5*rho*cda*np.linalg.norm(va)*va
        for acceleration in [0., .5, -.5]:
            # Gravity world -z; aggregate rotor force F=M*a-M*gvec-D.
            force = mass*np.array([acceleration,0.,g])-drag
            horizontal = float(np.linalg.norm(force[:2]))
            wind_rows.append(dict(wind_case=label,ground_speed_m_s=3.,
                wind_x_m_s=wind[0],wind_y_m_s=wind[1],air_speed_m_s=float(np.linalg.norm(va)),
                acceleration_ground_x_m_s2=acceleration,drag_x_N=float(drag[0]),drag_y_N=float(drag[1]),
                required_aggregate_force_x_N=float(force[0]),required_aggregate_force_y_N=float(force[1]),
                required_aggregate_force_norm_N=float(np.linalg.norm(force)),
                aggregate_force_tilt_deg=math.degrees(math.atan2(horizontal,force[2]))))
    write_csv('wind_ground_airspeed_force.csv', wind_rows)
    plt.rcParams.update({'font.family':['Noto Sans CJK JP','DejaVu Sans'],'font.size':10})
    speeds=np.linspace(0.,5.,401)
    fig, ax=plt.subplots(figsize=(7,4),layout='constrained')
    ax.plot(speeds,[delay*v+v*v/(2*a) for v in speeds],label='瞬时建立减速度：vτ+v²/(2a)')
    ax.plot(speeds,[delay*v+brake(v,a,j)['distance_m'] for v in speeds],label='有限急动度：vτ+S形制动距离')
    ax.set(xlabel='初始地速（m/s）',ylabel='理想运动学停止距离（m）',
           title='假设 a=0.5 m/s²、j=1 m/s³、τ=0.1 s；非实飞安全包线')
    ax.legend();ax.grid(alpha=.2)
    fig.savefig(ROOT/'figures/night_jerk_braking.png',dpi=180);plt.close(fig)
    result=dict(generated_utc=datetime.now(timezone.utc).isoformat(),
        scope='Analytic kinematics and aggregate force only. No coupled flight or safety certification.',
        assumptions=dict(initial_acceleration_m_s2=0.,terminal_acceleration_m_s2=0.,
                         constant_speed_during_delay=True,acceleration_limit_m_s2=a,jerk_limit_m_s3=j,
                         delay_s=delay,space_reserve_m=reserve,mass_kg=mass,density_kg_m3=rho,
                         aggregate_drag_CdA_m2=cda,wind_constant_for_force_snapshots=True),
        polynomial_and_quadrature_max_absolute_difference_m=max_error,
        existing_motion_profile_crosscheck='passed for all 7 tabulated positive speeds',
        budget_examples=budgets,
        running_model_sha256=hashlib.sha256((ROOT/'simulation/coupled_model.py').read_bytes()).hexdigest(),
        runtime_model_modified=False,
        limitations=['No initial positive acceleration, gust evolution, estimator error, actuator saturation, cable swing or impact in stopping-distance calculation.',
                     'The chosen acceleration limit is an input, not inferred from thrust-to-weight ratio.',
                     'Aggregate force balance does not enforce per-vehicle, per-motor, cable tension or attitude constraints.',
                     'Drag area and mass are hypothetical B0 aggregate values, not manufacturer-identified parameters.'])
    (ROOT/'results/checkpoint_04_braking_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
