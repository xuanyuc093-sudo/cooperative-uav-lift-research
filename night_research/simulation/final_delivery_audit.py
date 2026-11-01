"""Post-deadline consistency checks only. Does not run or resume simulation."""
from pathlib import Path
from datetime import datetime, timezone
import collections
import csv
import hashlib
import json
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
ORIGINAL=ROOT.parent/'uav_lift_report'
if not ORIGINAL.exists():ORIGINAL=ROOT.parent/'original_v1'
DEADLINE=datetime(2026,10,31,22,tzinfo=timezone.utc)


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    status=json.loads((ROOT/'results/EXPERIMENT_STATUS.json').read_text())
    manifest=json.loads((ROOT/'results/RUN_MANIFEST.json').read_text())
    assert datetime.now(timezone.utc)>=DEADLINE
    assert status['running'] is False
    cases=[];violations=[];trace_rows=[];file_hashes=[]
    for p in sorted((ROOT/'results/cases').glob('*.json')):
        r=json.loads(p.read_text());cases.append(r)
        file_hashes.append(dict(path=str(p.relative_to(ROOT)),sha256=sha(p)))
        if r['model_sha256']!=manifest['model_sha256']:violations.append([r['case_id'],'model version mismatch'])
        expected=hashlib.sha256(json.dumps({'config':r['config'],'model_hash':r['model_sha256']},sort_keys=True).encode()).hexdigest()[:20]
        if expected!=r['case_id']:violations.append([r['case_id'],'case id mismatch'])
        if datetime.fromisoformat(r['started_utc'])>=DEADLINE:violations.append([r['case_id'],'started after deadline'])
        m=r.get('metrics')
        if m is None:continue
        if m['config']!=r['config']:violations.append([r['case_id'],'config mismatch'])
        conditions=dict(run_completed=m['completed'],rmse_below_010m=m['position_rmse_m']<=.1,
            position_peak_below_025m=m['position_peak_error_m']<=.25,
            payload_attitude_below_10deg=m['attitude_peak_error_deg']<=10,
            cable_direction_change_below_10deg=m['cable_deviation_peak_deg']<=10,
            tension_above_03N=m['tension_min_N']>=.3,tension_below_10N=m['tension_peak_N']<=10,
            force_below_configured_cap=m['force_peak_N']<=r['config']['force_cap']+1e-6,
            effective_force_tilt_below_cap=m['force_tilt_peak_deg']<=r['config']['tilt_deg']+1e-6,
            vehicle_separation_above_045m=m['min_vehicle_distance_m']>=.45)
        if conditions!=r['screen_conditions'] or all(conditions.values())!=r['research_screen_met']:
            violations.append([r['case_id'],'screening mismatch'])
        if m['tension_min_N']<0:violations.append([r['case_id'],'negative tension'])
        if m['completed'] and abs(m['simulation_end_s']-m['intended_duration_s'])>r['config']['dt']+1e-8:
            violations.append([r['case_id'],'completed duration mismatch'])
        if r.get('fine_step_agreement'):
            fine=r['fine_step_metrics'];diff={k:abs(m[k]-fine[k]) for k in ['position_rmse_m','attitude_peak_error_deg','tension_peak_N','min_vehicle_distance_m']}
            if not (fine['completed'] and r['research_screen_met'] and r['fine_step_screen_met'] and
                    diff['position_rmse_m']<.002 and diff['attitude_peak_error_deg']<.5 and
                    diff['tension_peak_N']<.05 and diff['min_vehicle_distance_m']<.005):
                violations.append([r['case_id'],'invalid fine step label'])
            if fine['config']!=dict(r['config'],dt=r['config']['dt']/2):
                violations.append([r['case_id'],'fine config changes beyond dt'])
        trace=ROOT/r['trace']
        if not trace.exists():violations.append([r['case_id'],'missing trace']);continue
        with np.load(trace,allow_pickle=False) as z:
            a=z['history'];columns=z['columns']
            valid=bool(a.ndim==2 and a.shape[1]==len(columns)==19 and len(a)>0 and
                       np.isfinite(a).all() and np.all(np.diff(a[:,0])>0) and
                       a[-1,0]<=m['simulation_end_s']+1e-8)
            if not valid:violations.append([r['case_id'],'trace shape/time/finite check failed'])
            trace_rows.append(dict(case_id=r['case_id'],path=r['trace'],rows=len(a),columns=len(columns),
                final_stored_time_s=float(a[-1,0]),reported_end_s=m['simulation_end_s'],sha256=sha(trace)))
    ids={r['case_id'] for r in cases}
    with (ROOT/'results/experiment_summary.csv').open(encoding='utf-8-sig') as f:
        summary=list(csv.DictReader(f))
    if {r['case_id'] for r in summary}!=ids:violations.append(['summary','CSV/cases id mismatch'])
    stages=dict(collections.Counter(r['stage'] for r in cases))
    if status['aggregate']['records']!=len(cases) or status['aggregate']['stages']!=stages:
        violations.append(['status','final status/cases mismatch'])
    if sha(ROOT/'simulation/coupled_model.py')!=manifest['model_sha256']:
        violations.append(['source','model source hash changed'])
    original_checks=json.loads((ORIGINAL/'checksums.json').read_text())
    original_mismatches=[name for name,digest in original_checks.items() if sha(ORIGINAL/name)!=digest]
    if original_mismatches:violations.append(['original_v1','original checksum mismatch',original_mismatches])
    effective=set()
    for r in cases:
        c=dict(r['config'])
        if c['wind']==0:c.pop('seed')
        effective.add(json.dumps(c,sort_keys=True))
    previous=json.loads((ROOT/'results/checkpoint_05_analysis.json').read_text())
    coverage=[]
    for family in previous['queue_coverage']:
        rows=[r for r in cases if r['family']==family['family']]
        coverage.append(dict(family=family['family'],planned=family['planned_configurations'],
            committed=len(rows),not_submitted=family['planned_configurations']-len(rows),
            completed=sum(r['stage']=='integrated' for r in rows),
            early_guards=sum(r['stage']=='divergence_or_state_guard' for r in rows),
            initial_infeasible=sum(r['stage']=='initial_research_constraints_infeasible' for r in rows),
            deadline_interrupted=sum(r['stage']=='wall_clock_deadline' for r in rows)))
    deadline_cases=[dict(case_id=r['case_id'],started_utc=r['started_utc'],finished_utc=r['finished_utc'],
        simulation_end_s=r['metrics']['simulation_end_s'],intended_duration_s=r['metrics']['intended_duration_s'])
        for r in cases if r['stage']=='wall_clock_deadline']
    audit=dict(audited_utc=datetime.now(timezone.utc).isoformat(),research_deadline_utc=DEADLINE.isoformat(),
        experiment_closed_utc=status['updated_utc'],running=False,records=len(cases),stages=stages,
        complete_integrations=sum(r['stage']=='integrated' for r in cases),
        complete_integrations_not_meeting_research_screen=sum(r['stage']=='integrated' and not r.get('research_screen_met') for r in cases),
        research_screen_met_records=sum(bool(r.get('research_screen_met')) for r in cases),
        fine_step_agreement_records=sum(bool(r.get('fine_step_agreement')) for r in cases),
        effective_configuration_groups=len(effective),planned_queue=previous['planned_queue'],
        not_submitted=previous['planned_queue']-len(cases),queue_coverage=coverage,
        last_started_utc=max(r['started_utc'] for r in cases),last_finished_utc=max(r['finished_utc'] for r in cases),
        deadline_interrupted_cases=deadline_cases,source_sha256=manifest['model_sha256'],
        original_v1_checksum_mismatches=original_mismatches,trace_files_checked=len(trace_rows),
        case_file_hashes=file_hashes,integrity_violations=violations,
        check_scope='Post-deadline file, label and trace consistency only. No new simulation, research or safety certification.')
    (ROOT/'results/FINAL_DATA_AUDIT.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2))
    for name,rows in [('FINAL_TRACE_REGISTER.csv',trace_rows),('FINAL_QUEUE_COVERAGE.csv',coverage)]:
        with (ROOT/'results'/name).open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    assert not violations,violations
    print(json.dumps({k:v for k,v in audit.items() if k not in ['case_file_hashes','queue_coverage']},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
