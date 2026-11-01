"""Cross-check conclusion numbers against saved evidence; no simulation."""
from pathlib import Path
import argparse
import csv
import json

ROOT=Path(__file__).resolve().parents[1]
RESULTS=ROOT/'night_research/results'


def read(name):
    return json.loads((RESULTS/name).read_text())


def csv_rows(name):
    with (RESULTS/name).open(encoding='utf-8-sig',newline='') as f:
        return list(csv.DictReader(f))


def case(cid):
    r=read('cases/'+cid+'.json');m=r['metrics']
    return dict(case_id=cid,config=r['config'],completed=m['completed'],
        position_rmse_m=m['position_rmse_m'],position_peak_error_m=m['position_peak_error_m'],
        attitude_peak_error_deg=m['attitude_peak_error_deg'],force_peak_N=m['force_peak_N'],
        tension_min_N=m['tension_min_N'],min_vehicle_distance_m=m['min_vehicle_distance_m'],
        command_clipping_sample_fraction=m['command_clipping_time_fraction'],
        research_screen_met=r['research_screen_met'],fine_step_agreement=bool(r.get('fine_step_agreement')),
        failed_conditions=[k for k,v in r['screen_conditions'].items() if not v])


def build_evidence():
    masses=['950f3830dbe5452de4b0','9929008c2c5a2b972962','8932d217b00ad7749236','922090fb6a15b713073a']
    speed=['950f3830dbe5452de4b0','38d04abb765715ca5b28','01375e4489d23d0eeb6a','f9a0b817951a77695758','ed7c5e78a3da5bae8d28']
    wind_delay=['1d17bfba371dad6702f3','3ec7457a42a556a8bbd9','f194c412ce7fb59a6246','b2944506399c1433f24b','5a9eabc08fea65c6f195','6192e688a595f1f7d7b6']
    static=json.loads((ROOT/'original_v1/calculations.json').read_text())
    hover=next(x for x in static['hover'] if x['payload_kg']==.6)
    cfg=read('RUN_MANIFEST.json')['default_config']
    return dict(schema=1,research_cutoff_utc='2026-10-31T22:00:00Z',
        scope='Conclusions synthesized from existing evidence only. No new simulation or changed case label.',
        primary_recommendation=dict(type='engineering_recommendation_not_hardware_rating',
            nominal_payload_kg=.6,nominal_speed_m_s=1.,nominal_acceleration_m_s2=.5,
            nominal_jerk_m_s3=1.,topology='three non-collinear distributed attachments',
            first_control_route='Complete geometric control plus constrained cable/rotor allocation'),
        baseline_config=cfg,baseline_static=hover,
        elastic_initial_length_m=cfg['cable_length']+hover['tension_per_cable_N']/cfg['stiffness'],
        mass_comparison=[case(x) for x in masses],speed_comparison=[case(x) for x in speed],
        wind_delay_examples=[case(x) for x in wind_delay],
        force_reserves=[r for r in csv_rows('checkpoint_05_force_reserve.csv') if r['controller']=='independent'],
        braking_examples=[r for r in csv_rows('jerk_limited_braking.csv') if float(r['speed_ground_m_s']) in [1.,2.,3.,5.]],
        braking_assumptions=read('checkpoint_04_braking_audit.json')['assumptions'],
        braking_budget_examples=read('checkpoint_04_braking_audit.json')['budget_examples'],
        allocation_counterexample=read('checkpoint_03_independent_audit.json')['allocation'],
        topology_geometry=csv_rows('checkpoint_05_topology_geometry.csv'),
        direct_source_locations={'manufacturer_mass_and_performance':'original_v1/report.md sections 1-3',
            'motor_mass':'night_research/reports/01_evidence_and_identification.md N5',
            'control_architecture_evidence':'night_research/reports/03_braking_and_control_architectures.md B1-B4'},
        stored_cases_unchanged=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--write',action='store_true');args=parser.parse_args()
    expected=build_evidence();path=ROOT/'CONCLUSION_EVIDENCE.json'
    if args.write:path.write_text(json.dumps(expected,ensure_ascii=False,indent=2))
    actual=json.loads(path.read_text());assert expected==actual,'Conclusion evidence differs from saved records'
    assert all(x['fine_step_agreement'] for x in actual['mass_comparison'])
    assert [x['fine_step_agreement'] for x in actual['speed_comparison']]==[True,True,True,False,False]
    heavy=actual['mass_comparison'][-1]
    assert heavy['config']['payload_mass']==1.8 and heavy['command_clipping_sample_fraction']>.06
    assert actual['allocation_counterexample']['rotor_feasible'] is False
    body=(ROOT/'CONCLUSIONS.md').read_text()
    for value in [*(f"{x['force_peak_N']:.4f}" for x in actual['mass_comparison']),
                  *(f"{x['attitude_peak_error_deg']:.3f}" for x in actual['speed_comparison']),
                  *(f"{float(x['jerk_limited_distance_with_delay_m']):.2f}" for x in actual['braking_examples'])]:
        assert value in body,'Missing or inconsistent conclusion display: '+value
    assert f"{100*heavy['command_clipping_sample_fraction']:.3f}%" in body
    print(json.dumps({'passed':True,'mass_cases':4,'speed_cases':5,'wind_delay_cases':6,
        'scope':'Saved-evidence and displayed-number checks; no dynamics integration.'},ensure_ascii=False))


if __name__=='__main__':main()
