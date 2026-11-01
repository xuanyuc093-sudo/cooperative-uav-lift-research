"""Finite, resumable experiment queue. Stops at the user's wall-clock deadline.

The queue changes research assumptions, not identified real-aircraft parameters.
Results must never be interpreted as certified real-flight speed limits.
"""
from pathlib import Path
from dataclasses import asdict,replace
from datetime import datetime,timezone
import argparse
import csv
import hashlib
import itertools
import json
import os
import signal
import time
import traceback
import numpy as np
from coupled_model import Config,Model,run_case

ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'results'
CASES=RESULTS/'cases'
TRACES=RESULTS/'traces'
for d in [RESULTS,CASES,TRACES]:d.mkdir(exist_ok=True)

def utc():return datetime.now(timezone.utc).isoformat()

def atomic_json(path,data):
    temp=path.with_suffix(path.suffix+f'.{os.getpid()}.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False));temp.replace(path)

def screening(r):
    """Hypothetical research tolerances, not regulatory or manufacturer limits."""
    c=r['config']
    conditions={
      'run_completed':r['completed'],
      'rmse_below_010m':r['position_rmse_m']<=.10,
      'position_peak_below_025m':r['position_peak_error_m']<=.25,
      'payload_attitude_below_10deg':r['attitude_peak_error_deg']<=10.,
      'cable_direction_change_below_10deg':r['cable_deviation_peak_deg']<=10.,
      'tension_above_03N':r['tension_min_N']>=.3,
      'tension_below_10N':r['tension_peak_N']<=10.,
      'force_below_configured_cap':r['force_peak_N']<=c['force_cap']+1e-6,
      'effective_force_tilt_below_cap':r['force_tilt_peak_deg']<=c['tilt_deg']+1e-6,
      'vehicle_separation_above_045m':r['min_vehicle_distance_m']>=.45,
    }
    return conditions,all(conditions.values())

def queue():
    base=Config(dt=.001,control_dt=.01)
    seen=set()
    def configs(changes,family,seeds=(0,)):
        for change in changes:
            for seed,topology,controller in itertools.product(seeds,['common','offset_common','distributed'],['independent','cooperative']):
                c=replace(base,**dict(change,seed=seed,topology=topology,controller=controller))
                key=json.dumps(asdict(c),sort_keys=True)
                if key in seen:continue
                seen.add(key);yield c,family
    yield from configs([{}],'baseline',seeds=(0,1))
    for name,values in [
        ('payload_mass',[.3,1.,1.4,1.8]),
        ('speed',[.5,2.,3.,5.]),
        ('wind',[1.,3.,5.]),
        ('command_delay',[.02,.05,.1]),
        ('inclination_deg',[10.,35.,45.]),
        ('actuator_tau',[.02,.08,.12]),
        ('stiffness',[300.,3000.]),
        ('payload_mass_error',[-.2,.2]),
        ('force_cap',[9.,16.]),
        ('yaw_target_deg',[30.]),
    ]:
        yield from configs([{name:v} for v in values],'single_factor_'+name,seeds=(0,1))
    # Purposeful two-factor interactions precede a larger factorial queue.
    interactions=[]
    for wind,delay in itertools.product([1.,3.,5.],[.02,.05,.1]):interactions.append({'wind':wind,'command_delay':delay})
    for mass,speed in itertools.product([.3,1.,1.4,1.8],[.5,2.,3.,5.]):interactions.append({'payload_mass':mass,'speed':speed})
    for mass,wind in itertools.product([.3,1.,1.4,1.8],[1.,3.,5.]):interactions.append({'payload_mass':mass,'wind':wind})
    yield from configs(interactions,'two_factor',seeds=(0,1))
    combinations=list(itertools.product([.3,.6,1.,1.4,1.8],[0.,1.,3.,5.],[0.,.02,.05,.1],[.5,1.,2.,3.,5.]))
    rng=np.random.default_rng(20261031);rng.shuffle(combinations)
    for mass,wind,delay,speed in combinations:
        yield from configs([dict(payload_mass=mass,wind=wind,command_delay=delay,speed=speed)],'factorial',seeds=(0,1,2))

def summarize():
    rows=[]
    for file in sorted(CASES.glob('*.json')):
        try:d=json.loads(file.read_text())
        except (OSError,json.JSONDecodeError):continue
        row={k:d.get(k) for k in ['case_id','family','stage','model_sha256','started_utc','finished_utc']}
        if 'metrics' in d:
            m=d['metrics'];row.update({k:v for k,v in m.items() if k not in ['config','model_level']})
            row.update({'cfg_'+k:v for k,v in m['config'].items()})
        else:row.update({'config_'+k:v for k,v in d.get('config',{}).items()})
        row['research_screen_met']=d.get('research_screen_met',False)
        row['fine_step_rechecked']=d.get('fine_step_rechecked',False)
        row['fine_step_screen_met']=d.get('fine_step_screen_met',False)
        row['fine_step_agreement']=d.get('fine_step_agreement',False)
        row['caution']='reduced assumed-parameter model; no real-flight safety claim'
        rows.append(row)
    fields=sorted(set().union(*(x.keys() for x in rows))) if rows else []
    temp=RESULTS/('experiment_summary.csv.'+str(os.getpid())+'.tmp')
    with temp.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    temp.replace(RESULTS/'experiment_summary.csv')
    counts={}
    for r in rows:counts[r['stage']]=counts.get(r['stage'],0)+1
    stats={'updated_utc':utc(),'records':len(rows),'stages':counts,
           'research_screen_met_count':sum(bool(r['research_screen_met']) for r in rows),
           'fine_step_agreement_count':sum(bool(r['fine_step_agreement']) for r in rows),
           'scope':'Counts of simulated research tolerances. Do not equate these to safe flights.'}
    atomic_json(RESULTS/'AGGREGATE.json',stats)
    return stats


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--deadline',default='2026-10-31T22:00:00+00:00')
    parser.add_argument('--max-cases',type=int,default=20000);parser.add_argument('--summarize-only',action='store_true')
    args=parser.parse_args()
    if args.summarize_only:print(json.dumps(summarize(),indent=2));return
    gate=json.loads((RESULTS/'model_verification.json').read_text())
    if not gate.get('passed'):raise RuntimeError('Model verification gate has not passed')
    deadline=datetime.fromisoformat(args.deadline).timestamp()
    source=Path(__file__).with_name('coupled_model.py').read_bytes();model_hash=hashlib.sha256(source).hexdigest()
    snap=ROOT/'simulation'/('model_snapshot_'+model_hash[:12]+'.py')
    if not snap.exists():snap.write_bytes(source)
    manifest={'started_utc':utc(),'deadline_utc':args.deadline,'pid':os.getpid(),'model_sha256':model_hash,
              'model_snapshot':str(snap),'verification':gate,'default_config':asdict(Config(dt=.001,control_dt=.01)),
              'scope':'Reduced model only. No NMPC, no full attitude/rotor model, no real-flight guarantee.',
              'screen_thresholds':{'position_rmse_m':.1,'position_peak_m':.25,'attitude_peak_deg':10.,'cable_deviation_deg':10.,
                                   'tension_min_N':.3,'tension_max_N':10.,'vehicle_distance_min_m':.45},
              'metrics_sampling':'every integration step; traces downsampled to 50 Hz',
              'fine_step_rule':'recheck candidates meeting every research tolerance at half physics dt, same controller period',
              'stop_rule':'wall-clock deadline or STOP file; no fabricated result for unfinished cases'}
    atomic_json(RESULTS/'RUN_MANIFEST.json',manifest)
    consecutive_errors=0;done=0;why='queue_exhausted'
    for index,(cfg,family) in enumerate(queue()):
        if time.time()>=deadline or (ROOT/'STOP').exists():why='deadline_or_stop_file';break
        if done>=args.max_cases:why='case_limit';break
        key=json.dumps({'config':asdict(cfg),'model_hash':model_hash},sort_keys=True)
        cid=hashlib.sha256(key.encode()).hexdigest()[:20]
        output=CASES/(cid+'.json')
        if output.exists():continue
        before=utc();base={'case_id':cid,'family':family,'queue_index':index,'model_sha256':model_hash,'started_utc':before,'config':asdict(cfg)}
        atomic_json(RESULTS/'EXPERIMENT_STATUS.json',dict(running=True,pid=os.getpid(),updated_utc=utc(),case_id=cid,
                    family=family,cases_this_process=done,deadline_utc=args.deadline,model_sha256=model_hash))
        try:
            m=Model(cfg);y=m.initial();f0=np.linalg.norm(y[31:40].reshape(3,3),axis=1)
            xi=y[13:22].reshape(3,3);d0=min(np.linalg.norm(xi[i]-xi[j]) for i in range(3) for j in range(i))
            tilt0=np.degrees(np.arccos(np.clip(y[31:40].reshape(3,3)[:,2]/f0,-1,1)))
            if max(f0)>cfg.force_cap+1e-8 or max(tilt0)>cfg.tilt_deg+1e-8 or d0<.45:
                result=dict(base,stage='initial_research_constraints_infeasible',finished_utc=utc(),
                            initial_force_N=f0.tolist(),initial_tilt_deg=tilt0.tolist(),initial_min_distance_m=float(d0),
                            research_screen_met=False,explanation='No time integration: initial force/tilt/separation research constraints already violated.')
            else:
                trace=TRACES/(cid+'.npz')
                r=run_case(cfg,log_path=str(trace),deadline_epoch=deadline)
                conditions,met=screening(r)
                result=dict(base,stage='integrated' if r['completed'] else r['termination'],finished_utc=utc(),metrics=r,
                            screen_conditions=conditions,research_screen_met=met,fine_step_rechecked=False,
                            trace=str(trace.relative_to(ROOT)))
                if met and time.time()<deadline:
                    fine_cfg=replace(cfg,dt=cfg.dt/2)
                    fine=run_case(fine_cfg,deadline_epoch=deadline)
                    fine_conditions,fine_met=screening(fine)
                    diff={k:abs(r[k]-fine[k]) for k in ['position_rmse_m','attitude_peak_error_deg','tension_peak_N','min_vehicle_distance_m']}
                    agree=bool(fine['completed'] and fine_met and diff['position_rmse_m']<.002 and diff['attitude_peak_error_deg']<.5 and diff['tension_peak_N']<.05 and diff['min_vehicle_distance_m']<.005)
                    result.update(fine_step_rechecked=True,fine_step_metrics=fine,fine_step_screen_met=fine_met,
                                  fine_step_conditions=fine_conditions,fine_step_difference=diff,fine_step_agreement=agree,finished_utc=utc())
            atomic_json(output,result)
            consecutive_errors=0;done+=1
            with (RESULTS/'events.jsonl').open('a') as f:f.write(json.dumps({k:result.get(k) for k in ['case_id','family','stage','research_screen_met','fine_step_agreement','finished_utc']})+'\n')
            print(utc(),done,cid,family,result['stage'],'screen=',result['research_screen_met'],flush=True)
            if done%12==0:summarize()
        except Exception as e:
            consecutive_errors+=1
            result=dict(base,stage='implementation_exception',finished_utc=utc(),error=repr(e),traceback=traceback.format_exc())
            atomic_json(output,result);print(result['traceback'],flush=True)
            if consecutive_errors>=3:why='repeated_implementation_exceptions';break
    stats=summarize()
    atomic_json(RESULTS/'EXPERIMENT_STATUS.json',dict(running=False,pid=os.getpid(),updated_utc=utc(),termination=why,
                cases_this_process=done,deadline_utc=args.deadline,aggregate=stats,model_sha256=model_hash))
    print('FINISHED',why,json.dumps(stats),flush=True)

if __name__=='__main__':main()
