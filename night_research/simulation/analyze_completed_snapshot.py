"""Read-only analysis of an immutable 05:00 checkpoint of committed cases.

No experiment is started, stopped, or modified. Re-running reads the preserved
snapshot so the tables remain reproducible while the background batch advances.
"""
from pathlib import Path
from dataclasses import asdict
from datetime import datetime, timezone
import collections
import csv
import hashlib
import importlib.util
import json
import math
import os
import numpy as np
from scipy.optimize import brentq
from coupled_model import Config, Model
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'results'
SNAPSHOT=RESULTS/'checkpoint_05_records.json'
FACTORS=['payload_mass','speed','wind','command_delay','inclination_deg','actuator_tau',
         'stiffness','payload_mass_error','force_cap','yaw_target_deg']
TOPO={'common':'质心共点','offset_common':'偏置共点','distributed':'三点分布式'}
CTRL={'independent':'独立编队','cooperative':'载荷反馈分配'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def canonical(c):
    return json.dumps(c,sort_keys=True)


def write_csv(name,rows):
    with (RESULTS/name).open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else [])
        writer.writeheader();writer.writerows(rows)


def freeze():
    if SNAPSHOT.exists():
        return json.loads(SNAPSHOT.read_text())
    manifest=json.loads((RESULTS/'RUN_MANIFEST.json').read_text())
    model_hash=digest((ROOT/'simulation/coupled_model.py').read_bytes())
    assert model_hash==manifest['model_sha256']
    records=[];source_files=[]
    for p in sorted((RESULTS/'cases').glob('*.json')):
        raw=p.read_bytes();record=json.loads(raw)
        if record['model_sha256']!=model_hash:continue
        records.append(record)
        source_files.append(dict(path=str(p.relative_to(ROOT)),sha256=digest(raw)))
    snapshot=dict(time_utc=datetime.now(timezone.utc).isoformat(),model_sha256=model_hash,
                  runner_snapshot=manifest['runner_source_snapshot'],
                  runner_sha256=manifest['runner_source_at_launch_sha256'],
                  records=records,source_files=source_files,
                  scope='Committed records frozen for checkpoint analysis; no pending cases fabricated.')
    temp=SNAPSHOT.with_suffix('.json.'+str(os.getpid())+'.tmp')
    temp.write_text(json.dumps(snapshot,ensure_ascii=False,indent=2));temp.replace(SNAPSHOT)
    return snapshot


def geometry(topology,angle):
    cfg=Config(dt=.001,topology=topology,inclination_deg=angle)
    model=Model(cfg);state=model.initial()
    x,v,q,w,xi,vi,force=model.unpack(state)
    _,t,u,e=model.cable_forces(state)
    d=min(np.linalg.norm(xi[i]-xi[j]) for i in range(3) for j in range(i))
    fixed_direction_map=np.vstack([u.T,np.stack([np.cross(ri,ui) for ri,ui in zip(model.r,u)]).T])
    return dict(topology=topology,inclination_deg=angle,
        initial_min_vehicle_distance_m=float(d),initial_uav_height_m=float(xi[0,2]),
        initial_force_per_vehicle_N=float(np.linalg.norm(force[0])),
        initial_force_tilt_deg=math.degrees(math.atan2(np.linalg.norm(force[0,:2]),force[0,2])),
        tension_per_cable_N=float(t[0]),vector_force_map_rank=int(np.linalg.matrix_rank(model.G)),
        fixed_direction_scalar_tension_map_rank=int(np.linalg.matrix_rank(fixed_direction_map)))


def main():
    snapshot=freeze();records=snapshot['records'];base=asdict(Config(dt=.001,control_dt=.01))
    by_config={canonical(r['config']):r for r in records}
    assert len(by_config)==len(records)
    # The queue snapshot reproduces the actual ordering, including seed duplicates.
    runner_path=ROOT/snapshot['runner_snapshot']
    assert digest(runner_path.read_bytes())==snapshot['runner_sha256']
    spec=importlib.util.spec_from_file_location('checkpoint05_frozen_runner',runner_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    expected=list(module.queue())
    expected_by_family=collections.Counter(family for cfg,family in expected)
    observed_by_family=collections.Counter(r['family'] for r in records)
    coverage=[]
    for family,count in expected_by_family.items():
        subset=[r for r in records if r['family']==family]
        coverage.append(dict(family=family,planned_configurations=count,committed_records=len(subset),
            missing_at_checkpoint=count-len(subset),complete_integrations=sum(r['stage']=='integrated' for r in subset),
            early_guards=sum(r['stage']=='divergence_or_state_guard' for r in subset),
            initial_infeasible=sum(r['stage']=='initial_research_constraints_infeasible' for r in subset),
            fine_step_agreement=sum(bool(r.get('fine_step_agreement')) for r in subset)))
    write_csv('checkpoint_05_queue_coverage.csv',coverage)
    violations=[]
    for record in records:
        c=record['config'];m=record.get('metrics')
        cid=digest(json.dumps({'config':c,'model_hash':snapshot['model_sha256']},sort_keys=True).encode())[:20]
        if cid!=record['case_id']:violations.append([record['case_id'],'case id/config hash mismatch'])
        if m:
            if c!=m['config']:violations.append([record['case_id'],'metric config mismatch'])
            if m['tension_min_N']<0:violations.append([record['case_id'],'negative tension'])
            conditions,met=module.screening(m)
            if conditions!=record['screen_conditions'] or met!=record['research_screen_met']:
                violations.append([record['case_id'],'stored screening mismatch'])
            if m['completed'] and abs(m['simulation_end_s']-m['intended_duration_s'])>c['dt']+1e-8:
                violations.append([record['case_id'],'complete label/time mismatch'])
        if record.get('fine_step_agreement'):
            fine=record['fine_step_metrics'];diff={k:abs(m[k]-fine[k]) for k in ['position_rmse_m','attitude_peak_error_deg','tension_peak_N','min_vehicle_distance_m']}
            if not (fine['completed'] and module.screening(fine)[1] and record['research_screen_met']):
                violations.append([record['case_id'],'invalid fine-step pass label'])
            if any(abs(v-record['fine_step_difference'][k])>1e-12 for k,v in diff.items()):
                violations.append([record['case_id'],'fine-step difference mismatch'])
            if not (diff['position_rmse_m']<.002 and diff['attitude_peak_error_deg']<.5 and diff['tension_peak_N']<.05 and diff['min_vehicle_distance_m']<.005):
                violations.append([record['case_id'],'fine-step agreement threshold mismatch'])
            expected_fine=dict(c,dt=c['dt']/2)
            if fine['config']!=expected_fine:
                violations.append([record['case_id'],'fine-step config changes beyond dt'])
    assert not violations,violations
    # Exact config selection: all other fields, including seed, must be identical.
    def select(**changes):
        return by_config.get(canonical(dict(base,**changes)))
    pairs=[]
    for cfg,family in expected:
        c=asdict(cfg)
        if c['seed']!=0 or c['controller']!='independent':continue
        other=dict(c,controller='cooperative')
        a=by_config.get(canonical(c));b=by_config.get(canonical(other))
        if a is None and b is None:continue
        changed=[k for k in c if c[k]!=other[k]]
        assert changed==['controller']
        row=dict(family=family,topology=c['topology'],payload_mass_kg=c['payload_mass'],
                 target_speed_m_s=c['speed'],base_wind_m_s=c['wind'],command_delay_s=c['command_delay'],
                 changed_config_fields='controller',independent_case_id=a['case_id'] if a else '',
                 cooperative_case_id=b['case_id'] if b else '',pair_complete=a is not None and b is not None)
        for label,record in [('independent',a),('cooperative',b)]:
            row[label+'_stage']=record['stage'] if record else 'not_executed_at_checkpoint'
            row[label+'_fine_step_agreement']=bool(record and record.get('fine_step_agreement'))
            m=record.get('metrics') if record else None
            for metric in ['position_rmse_m','position_peak_error_m','attitude_peak_error_deg',
                           'force_peak_N','tension_min_N','slack_time_fraction','min_vehicle_distance_m']:
                row[label+'_'+metric]=m[metric] if m else None
        row['rmse_delta_cooperative_minus_independent_m']=(b['metrics']['position_rmse_m']-a['metrics']['position_rmse_m']) if a and b and a['stage']==b['stage']=='integrated' else None
        pairs.append(row)
    write_csv('checkpoint_05_matched_controller_pairs.csv',pairs)
    single=[]
    values={'payload_mass':[.3,.6,1.,1.4,1.8],'speed':[.5,1.,2.,3.,5.],
            'wind':[0.,1.,3.,5.],'command_delay':[0.,.02,.05,.1],
            'inclination_deg':[10.,20.,35.,45.],'actuator_tau':[.02,.04,.08,.12],
            'stiffness':[300.,1000.,3000.],'payload_mass_error':[-.2,0.,.2],
            'force_cap':[9.,12.8,16.],'yaw_target_deg':[0.,30.]}
    for factor in FACTORS:
        for controller in ['independent','cooperative']:
            for value in values[factor]:
                record=select(topology='distributed',controller=controller,**{factor:value})
                row=dict(factor=factor,value=value,controller=controller,topology='distributed',seed=0,
                         case_id=record['case_id'] if record else '',stage=record['stage'] if record else 'missing',
                         fine_step_agreement=bool(record and record.get('fine_step_agreement')))
                m=record.get('metrics') if record else None
                for key in ['position_rmse_m','position_peak_error_m','attitude_peak_error_deg','force_peak_N',
                            'tension_min_N','tension_peak_N','command_clipping_time_fraction','slack_time_fraction',
                            'min_vehicle_distance_m','reference_distance_m','intended_duration_s',
                            'load_speed_peak_m_s','reference_acceleration_m_s2','reference_jerk_m_s3']:
                    row[key]=m[key] if m else None
                row['failed_research_conditions']=';'.join(k for k,v in record.get('screen_conditions',{}).items() if not v) if record else ''
                single.append(row)
    write_csv('checkpoint_05_distributed_single_factors.csv',single)
    mass_speed_grid=[]
    for controller in CTRL:
        for mass in [.3,.6,1.,1.4,1.8]:
            for speed in [.5,1.,2.,3.,5.]:
                record=select(topology='distributed',controller=controller,payload_mass=mass,speed=speed)
                mass_speed_grid.append(dict(controller=controller,payload_mass_kg=mass,target_speed_m_s=speed,
                    stage=record['stage'] if record else 'not_executed_at_checkpoint',
                    case_id=record['case_id'] if record else '',fine_step_agreement=bool(record and record.get('fine_step_agreement'))))
    write_csv('checkpoint_05_mass_speed_grid.csv',mass_speed_grid)
    reserves=[]
    for row in single:
        if row['factor']!='payload_mass' or row['stage']!='integrated':continue
        ratio=3*base['force_cap']/((3*base['vehicle_mass']+row['value'])*base['gravity'])
        reserves.append(dict(controller=row['controller'],payload_mass_kg=row['value'],
            configured_aggregate_thrust_to_weight_ratio=ratio,
            force_cap_per_vehicle_N=base['force_cap'],actual_force_peak_per_vehicle_N=row['force_peak_N'],
            minimum_unused_norm_cap_N=base['force_cap']-row['force_peak_N'],
            any_vehicle_command_clipping_sample_fraction=row['command_clipping_time_fraction'],
            fine_step_agreement=row['fine_step_agreement'],case_id=row['case_id'],
            scope='Assumed vector force cap; not identified continuous or peak hardware rating.'))
    write_csv('checkpoint_05_force_reserve.csv',reserves)
    yaw_geometric_change=math.degrees(math.acos(math.cos(math.radians(20))**2+
        math.sin(math.radians(20))**2*math.cos(math.radians(30))))
    yaw_rows=[]
    for record in records:
        c=record['config']
        if c['yaw_target_deg']!=30 or c['seed']!=0:continue
        m=record.get('metrics',{})
        scope=('structural_full_pose_task_limitation' if c['topology']!='distributed'
               else 'review_initial_direction_threshold_for_intended_maneuver')
        yaw_rows.append(dict(case_id=record['case_id'],topology=c['topology'],controller=c['controller'],
            original_research_screen_met=record.get('research_screen_met',False),
            attitude_peak_error_deg=m.get('attitude_peak_error_deg'),
            cable_direction_change_from_initial_peak_deg=m.get('cable_deviation_peak_deg'),
            analysis_v2_scope_annotation=scope,
            altered_original_result=False))
    write_csv('checkpoint_05_yaw_scope_v2.csv',yaw_rows)
    (RESULTS/'checkpoint_05_metric_scope_diagnostics.json').write_text(json.dumps(dict(
        snapshot_utc=snapshot['time_utc'],classification_version='analysis_v2_scope_annotations_only',
        ideal_20deg_cable_rotated_30deg_about_vertical_direction_change_deg=yaw_geometric_change,
        geometric_example_scope='One intended rotating formation, not a proof that every 30-degree load yaw requires this cable motion.',
        limitation='The stored trace does not contain cable directions. A new reference-relative cable error cannot be reconstructed from the trace alone.',
        preserved_original_screen_flags=True,updated_runtime_model=False,
        revised_interpretation='Retain all original outcomes. Diagnose structural task mismatch and the meaning of an initial-direction threshold separately; no safety pass is awarded.'),ensure_ascii=False,indent=2))
    geometry_rows=[dict(comparison='fixed_angle_and_length',**geometry(t,20.)) for t in TOPO]
    desired_spacing=geometry('distributed',20.)['initial_min_vehicle_distance_m']
    for topology in TOPO:
        angle=brentq(lambda th:geometry(topology,th)['initial_min_vehicle_distance_m']-desired_spacing,1.,70.)
        geometry_rows.append(dict(comparison='matched_initial_vehicle_spacing_static_only',**geometry(topology,angle)))
    write_csv('checkpoint_05_topology_geometry.csv',geometry_rows)
    plt.rcParams.update({'font.family':['Noto Sans CJK JP','DejaVu Sans'],'font.size':9})
    fig,axes=plt.subplots(2,2,figsize=(9,6),layout='constrained')
    metrics=['position_rmse_m','attitude_peak_error_deg','force_peak_N','load_speed_peak_m_s']
    labels=['载荷位置RMSE（m）','载荷姿态峰值（°）','单机合力峰值（N）','实际载荷速度峰值（m/s）']
    for controller in CTRL:
        points=[p for p in single if p['factor']=='speed' and p['controller']==controller and p['stage']=='integrated']
        for ax,metric,label in zip(axes.flat,metrics,labels):
            ax.plot([p['value'] for p in points],[p[metric] for p in points],'o-',label=CTRL[controller])
            ax.set(xlabel='目标峰值地速（m/s）',ylabel=label);ax.grid(alpha=.2)
    axes[0,1].axhline(10,color='gray',ls='--',label='研究阈值10°')
    axes[1,0].axhline(12.8,color='gray',ls='--',label='假设合力上限')
    axes[1,1].plot([.5,1,2,3,5],[.5,1,2,3,5],color='gray',ls='--',label='目标=实际峰值')
    axes[0,0].legend();axes[1,0].legend();axes[1,1].legend()
    fig.suptitle('分布式、无风、零延迟、seed=0；固定加速度/急动度，路径随速度改变\n只比较两个固定示范实现；失败点未统一细化，不作算法家族排名')
    fig.savefig(ROOT/'figures/night_speed_paired.png',dpi=180);plt.close(fig)
    gridrows=[];colors=['#d9d9d9','#a7d7ac','#f2d092','#e79f9d','#9ebcdb']
    legend=['未执行','完成且细化一致','完成但研究判据未通过','提前状态保护终止','初始不可行/其他']
    fig,axes=plt.subplots(1,2,figsize=(9,4.3),layout='constrained')
    for ax,controller in zip(axes,CTRL):
        grid=np.zeros((4,4),dtype=int)
        for wi,wind in enumerate([0.,1.,3.,5.]):
            for di,delay in enumerate([0.,.02,.05,.1]):
                record=select(topology='distributed',controller=controller,wind=wind,command_delay=delay)
                code=0 if record is None else (1 if record.get('fine_step_agreement') else (2 if record['stage']=='integrated' else (3 if record['stage']=='divergence_or_state_guard' else 4)))
                grid[wi,di]=code
                ax.text(di,wi,['缺','细化','未过','中止','其他'][code],ha='center',va='center',fontsize=9)
                gridrows.append(dict(controller=controller,wind_m_s=wind,command_delay_s=delay,classification=legend[code],case_id=record['case_id'] if record else ''))
        ax.imshow(grid,cmap=ListedColormap(colors),vmin=-.5,vmax=4.5)
        ax.set(xticks=range(4),xticklabels=['0','20','50','100'],yticks=range(4),yticklabels=['0','1','3','5'],
               xlabel='指令延迟（ms）',ylabel='基准侧风（m/s；瞬时可达1.2倍）',title=CTRL[controller])
    fig.legend(handles=[Patch(color=c,label=l) for c,l in zip(colors,legend)],loc='outside lower center',ncol=3,fontsize=8)
    fig.suptitle('分布式、载荷0.6 kg、目标1 m/s、seed=0；每格对应已保存的明确工况')
    fig.savefig(ROOT/'figures/night_wind_delay_grid.png',dpi=180);plt.close(fig)
    write_csv('checkpoint_05_wind_delay_grid.csv',gridrows)
    effective=set()
    for record in records:
        c=dict(record['config'])
        if c['wind']==0:c.pop('seed')
        effective.add(canonical(c))
    result=dict(snapshot_utc=snapshot['time_utc'],snapshot_file=SNAPSHOT.name,
        snapshot_sha256=digest(SNAPSHOT.read_bytes()),records=len(records),
        effective_configuration_groups=len(effective),planned_queue=len(expected),
        missing_configurations_at_checkpoint=len(expected)-len(records),
        stages=dict(collections.Counter(r['stage'] for r in records)),
        fine_step_agreement_records=sum(bool(r.get('fine_step_agreement')) for r in records),
        current_source_matches_snapshot=digest((ROOT/'simulation/coupled_model.py').read_bytes())==snapshot['model_sha256'],
        integrity_violations=violations,queue_coverage=coverage,
        complete_controller_pairs=sum(row['pair_complete'] for row in pairs),controller_pair_rows=len(pairs),
        topology_geometry=geometry_rows,
        interpretation=['No probability or field safety inference from this parameter queue.',
            'A passed screen does not require zero command clipping or a positive unused force reserve.',
            'Both controllers use exact model cable force as feedforward; no sensing/estimation errors are modeled.',
            'Wind seed changes phase only; no-wind repetitions are not independent uncertainty trials.',
            'Geometry comparison at equal spacing is static only; no new matched-spacing flight simulation run.'])
    (RESULTS/'checkpoint_05_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
