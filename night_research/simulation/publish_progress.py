"""Publish only committed experiment records; no invented pending results."""
from pathlib import Path
from datetime import datetime,timezone,timedelta
from dataclasses import asdict
from zipfile import ZipFile,ZIP_DEFLATED
import argparse
import collections
import hashlib
import json
import subprocess
import sys
import fcntl
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from coupled_model import Config
from run_batch import summarize

ROOT=Path(__file__).resolve().parents[1]
ORIGINAL=ROOT.parent/'uav_lift_report'
if not ORIGINAL.exists():ORIGINAL=ROOT.parent/'original_v1'
plt.rcParams.update({'font.family':['Noto Sans CJK JP','DejaVu Sans'],'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
DEADLINE=datetime(2026,10,31,22,tzinfo=timezone.utc)
STAGE_NAMES={'integrated':'完成时间积分','divergence_or_state_guard':'状态保护终止','wall_clock_deadline':'截止中止','implementation_exception':'实现异常','initial_research_constraints_infeasible':'初始研究约束不可行'}

def number(x,digits=3):
    return f'{x:.{digits}f}' if isinstance(x,(float,int)) else str(x)

def table(headers,rows):
    return '\n'.join(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|']+
                     ['|'+'|'.join(str(v).replace('|','/') for v in row)+'|' for row in rows])+'\n'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--final',action='store_true');parser.add_argument('--export',action='store_true')
    args=parser.parse_args()
    # A scheduled review and the local supervisor may arrive together. Serialize
    # publication without locking the independent experiment writer.
    publication_lock=(ROOT/'.publication.lock').open('w')
    fcntl.flock(publication_lock,fcntl.LOCK_EX)
    now=datetime.now(timezone.utc);final=args.final and now>=DEADLINE
    name='overnight_report' if final else 'overnight_progress'
    stats=summarize()
    manifest=json.loads((ROOT/'results'/'RUN_MANIFEST.json').read_text())
    verification=json.loads((ROOT/'results'/'model_verification.json').read_text())
    records=[]
    for file in (ROOT/'results'/'cases').glob('*.json'):
        try:records.append(json.loads(file.read_text()))
        except (OSError,json.JSONDecodeError):continue
    current=[r for r in records if r.get('model_sha256')==manifest['model_sha256']]
    integrated=[r for r in current if 'metrics'in r]
    complete=[r for r in integrated if r['metrics']['completed']]
    old=len(records)-len(current)
    local=now.astimezone(timezone(timedelta(hours=8)))
    display_local=DEADLINE.astimezone(timezone(timedelta(hours=8))) if final else local
    title='多架无人机联合吊运：夜间补充研究'
    lines=['---',f'title: "{title}"',
           'subtitle: "原始证据补强、参数辨识与可复算耦合模型"',
           f'date: "研究截至 {display_local:%Y-%m-%d %H:%M:%S} UTC+8 · '+('截止汇总' if final else '运行中阶段快照')+'"',
           'lang: zh-CN','toc-title: "目录"','---','',
           '# 本轮实际完成范围','',
           f'原报告17页、24项资料保留不变。本轮新增原始来源与独立审查专题、可运行的三维耦合简化模型，以及逐工况保存的研究参数扫描。当前读取到**{len(current)}条本版本记录**，其中{len(complete)}条完成了预定时间积分。候选工况的细步长复核另行记录。',
           '', '**本报告没有给出真实机型的安全最高速度。**模型参数含明确假设，且省略无人机自身姿态和单旋翼分配。这里的“研究判据满足”只针对所定义的模型与阈值；不等同于实飞安全、制造商额定性能或法规认证。','',
           f'执行截止：2026年11月1日06:00 UTC+8。当前模型SHA-256前16位：`{manifest["model_sha256"][:16]}`，完整值见RUN_MANIFEST.json。其他模型版本记录数：{old}；不混入本版本的比较。','',
           ((ROOT/'FINAL_SUMMARY.md').read_text()+'\n\n'+f'终稿导出时间：{local:%Y-%m-%d %H:%M:%S} UTC+8。截止之后只进行既有材料汇总、导出与一致性核验。\n') if final and (ROOT/'FINAL_SUMMARY.md').exists() else '',
           '# 一、主要任务的证据补强','',
           (ROOT/'reports'/'01_evidence_and_identification.md').read_text(),'',
           '\n\n'.join(f.read_text() for f in sorted((ROOT/'reports').glob('*.md')) if not f.name.startswith('01_')),'',
           '# 二、实际运行的模型、控制器与数值核查','',
           '## 1. 模型层级与明确省略项','',
           '状态包含载荷位置/速度、姿态四元数与体角速度、三机位置/速度和三机合力向量，共40个标量。载荷是刚体；无人机暂作质点；绳索是无质量、只受拉的弹性阻尼连接。',
           '',
           r'载荷所受每绳张力为$t_i=\max(0,k(d_i-l_i)+c\dot d_i)$，但只有$d_i>l_i$时才激活；松绳时为零。无人机受到相反的绳索力；载荷转矩由各挂点力臂计算。用RK4积分并归一化四元数。初始化采用$d_i=l_i+t_{i,0}/k$，保留静态预应变。',
           '',
           '没有模拟无人机本体姿态、单电机力矩分配、电池内阻/热、旋翼尾流、地面接触、绳材破断、绳间碰撞或不可伸长绳的瞬时冲击定律。一阶合力响应也不能保证构成实际四旋翼能力的严格上界。因此该模型适于检查耦合现象和实现问题，不能代替完整飞控或实飞验证。','',
           '## 2. 两个示范控制器','',
           '- 独立编队跟踪：各机跟踪固定参考编队位置，采用相同的绳索力前馈和合力执行器假设。',
           '- 载荷反馈分配：由载荷位置/姿态误差生成期望合力与力矩，使用几何力映射伪逆和零空间偏好分配到各绳方向，再由各机位置环跟踪。它是示范实现，没有宣称完整复现某篇论文，也没有实现NMPC。',
           '',
           '两者使用理想状态信息，并将模型真实瞬时绳力作为精确前馈；没有传感噪声、绳力估计误差或时间戳不同步，也没有按每个载荷/风况重新优化增益。对比反映此固定实现与工况的关系，不可据此给整个算法家族排名。共点挂载的任意偏航任务本来不可实现，应归为任务/拓扑不匹配，而不是控制器调参失败。','',
           '## 3. 数值核查','',
           table(['检查','已得到的结果'],[
               ['三种拓扑初始静力','各状态导数残差最大约'+number(max(verification['static_equilibrium_max_state_derivative'].values()),14)],
               ['保守系统能量','0.25 ms步长、2 s释放试验，相对漂移峰值'+f"{verification['conservative_system']['relative_energy_drift_max']:.3g}"],
               ['线动量漂移','峰值'+f"{verification['conservative_system']['linear_momentum_drift_max']:.3g}"+' kg·m/s'],
               ['角动量漂移','峰值'+f"{verification['conservative_system']['angular_momentum_drift_max']:.3g}"+' kg·m²/s'],
               ['阻尼系统','已检查机械能不增加'],
               ['松绳','已检查松弛绳不产生压缩力'],
               ['名义步长细化','2 ms至1 ms的位置RMSE变化'+f"{verification['fine_step_rmse_change_m']:.3g}"+' m；控制周期保持不变']]),
           '',
           '保守系统最初在1 ms步长下没有通过预定能量误差阈值，结果约4.26×10⁻⁴；细化后才降低。原始失败和细化序列保留在conservation_diagnostics.json。名义工况收敛不等于所有松绳/受扰工况都已收敛；批量计算中满足全部初筛阈值的候选点会另用一半积分步长复算。','',
           '## 4. 轨迹、风与时延口径','',
           '采用七段S曲线，速度、加速度和急动度分别设限；名义加速度0.5 m/s²、急动度1 m/s³。提高速度时会相应增加运动距离，保留至少2 s匀速段，而不是把相同路径强行压缩时间后把加速度影响全归于“速度”。例如参考峰值1 m/s对应4.5 m运动，5 m/s对应62.5 m。',
           '',
           '风是沿世界y轴的侧风，4 s内建立，叠加20%的正弦变化；配置中的wind是基准幅值，不是全过程峰值。command_delay仅为指令传输延迟，不等于完整状态估计/通信网络模型。物理峰值与松绳在每个积分步检查，存储时序下采样为50 Hz。','',
           '# 三、当前实验结果','',
           '## 1. 记录与分类','',
           table(['类别','数量'],[(STAGE_NAMES.get(k,k),v) for k,v in sorted(collections.Counter(r['stage'] for r in current).items())]),
           '',
           f'粗步长满足全部研究阈值的记录：{sum(bool(r.get("research_screen_met")) for r in current)}；经细步长复算且满足一致性阈值的记录：{sum(bool(r.get("fine_step_agreement")) for r in current)}。这个比例只描述当前有限队列，不是随机事故概率或真实系统可靠性估计。未执行的网格点不得视为通过或失败。','',
           '研究阈值为：位置RMSE≤0.10 m、峰值≤0.25 m；载荷姿态误差≤10°；绳方向相对初始偏差≤10°；张力0.3–10 N；有效合力倾角≤25°；各机推力≤配置上限；机间中心距离≥0.45 m。它们都是研究设计假设。','',
           '这些判据没有要求指令裁剪比例为零或保留额外动力余量。slack_time_fraction统计任意一绳张力低于1e-6 N的采样比例，可能包括近零张力，不等于逐绳几何松弛时间之和；command_clipping_time_fraction统计任意一机合力指令因范数/倾角限制被裁剪的采样比例，不是单电机硬件饱和测量。初始方向偏差阈值还需检查与计划转向任务是否相容，详见专题四。','',
           '细化一致性阈值为：位置RMSE差小于2 mm、姿态峰值差小于0.5°、张力峰值差小于0.05 N、最小机间距差小于5 mm，并要求两次都完成且满足研究阈值。','',
           '## 2. 基准工况','']
    topo={'common':'质心共点','offset_common':'偏置共点','distributed':'三点分布式'}
    ctrl={'independent':'独立编队','cooperative':'载荷反馈分配'}
    baselines=sorted([r for r in complete if r['family']=='baseline' and r['metrics']['config']['seed']==0],key=lambda r:(r['metrics']['config']['topology'],r['metrics']['config']['controller']))
    lines.append(table(['拓扑/控制','位置RMSE m','姿态峰值°','最小张力N','最小间距m','细化一致'],[
        [topo[r['metrics']['config']['topology']]+'/'+ctrl[r['metrics']['config']['controller']],
         number(r['metrics']['position_rmse_m']),number(r['metrics']['attitude_peak_error_deg']),
         number(r['metrics']['tension_min_N']),number(r['metrics']['min_vehicle_distance_m']),
         '是' if r.get('fine_step_agreement') else '否/待完成'] for r in baselines]))
    lines+=['','## 3. 单因素与失败结果','']
    factors=['payload_mass','speed','wind','command_delay','inclination_deg','actuator_tau','stiffness','payload_mass_error','force_cap','yaw_target_deg']
    selected=[]
    for f in factors:
        points=[r for r in complete if r['family']=='single_factor_'+f and r['metrics']['config']['topology']=='distributed' and r['metrics']['config']['controller']=='cooperative' and r['metrics']['config']['seed']==0]
        for r in sorted(points,key=lambda r:r['metrics']['config'][f]):
            m=r['metrics'];selected.append([f,number(m['config'][f]),number(m['position_rmse_m']),number(m['attitude_peak_error_deg']),number(m['tension_min_N']),
                                          '满足' if r.get('fine_step_agreement') else ('粗步满足/细化待定' if r.get('research_screen_met') else '未满足')])
    lines.append(table(['因素','取值','RMSE m','姿态峰值°','最小张力N','本模型判据'],selected))
    lines+=['','以下列出完成积分记录中位置峰值偏差较大的若干工况，保留它们是为了发现模型或控制器的限制。表中的目标速度不表示实际跟踪已达到该速度。','']
    worst=sorted(complete,key=lambda r:r['metrics']['position_peak_error_m'],reverse=True)[:8]
    lines.append(table(['拓扑/控制','载荷kg','基准侧风m/s','延迟s','目标速度m/s','位置峰值m'],[
        [topo[r['metrics']['config']['topology']]+'/'+ctrl[r['metrics']['config']['controller']],
         number(r['metrics']['config']['payload_mass']),number(r['metrics']['config']['wind']),
         number(r['metrics']['config']['command_delay']),number(r['metrics']['config']['speed']),
         number(r['metrics']['position_peak_error_m'])] for r in worst]))
    guards=sorted([r for r in integrated if not r['metrics']['completed']],key=lambda r:r['metrics']['position_peak_error_m'],reverse=True)[:8]
    if guards:
        lines+=['','提前终止的工况另外列出；终止前RMSE与峰值覆盖的时间较短，不与完整任务RMSE直接排名。','']
        lines.append(table(['拓扑/控制','目标速度m/s','基准风m/s','终止时刻s','终止前位置峰值m','原因'],[
            [topo[r['metrics']['config']['topology']]+'/'+ctrl[r['metrics']['config']['controller']],
             number(r['metrics']['config']['speed']),number(r['metrics']['config']['wind']),
             number(r['metrics']['simulation_end_s']),number(r['metrics']['position_peak_error_m']),STAGE_NAMES.get(r['stage'],r['stage'])] for r in guards]))
    deterministic=set()
    for r in integrated:
        c=dict(r['metrics']['config'])
        if c['wind']==0:c.pop('seed',None)
        deterministic.add(json.dumps(c,sort_keys=True))
    lines+=['',f'已积分记录对应的有效配置组数（无风时不把不同seed算作独立随机样本）：{len(deterministic)}。seed仅改变风相位；这些记录不代表独立硬件试验。','']
    lines+=['','## 4. 时序示例','']
    # Explicitly label different conditions; these traces are not a fair head-to-head
    # algorithm ranking.
    traces=[]
    stable=next((r for r in baselines if r['metrics']['config']['topology']=='distributed' and r['metrics']['config']['controller']=='cooperative'),None)
    if stable:traces.append((ROOT/stable['trace'],'分布式基准：无风、无延迟、100 Hz控制'))
    severe=ROOT/'results'/'exploratory_wind3_delay04_verify_dt_0.0005.npz'
    if severe.exists():traces.append((severe,'探索失败：侧风3 m/s、40 ms延迟、50 Hz控制'))
    if traces:
        fig,ax=plt.subplots(2,1,figsize=(9,6),layout='constrained')
        for trace,label in traces:
            a=np.load(trace)['history'];t=a[:,0]
            ax[0].plot(t,np.linalg.norm(a[:,1:4]-a[:,4:7],axis=1),label=label)
            ax[1].plot(t,a[:,16],label=label)
        ax[0].set(ylabel='位置误差（m）',title='不同工况的时序示例：用于展示边界，不作算法排名');ax[0].legend(fontsize=8)
        ax[1].set(xlabel='仿真时间（s）',ylabel='载荷姿态误差（度）')
        fig.savefig(ROOT/'figures'/'night_traces.png',dpi=180);plt.close(fig)
        lines+=['![模型时序：失败工况与基准工况的控制更新率、扰动和时延不同。](figures/night_traces.png)','']
    lines+=['# 四、结论的适用边界与下一步','',
            '本轮把原报告的静力初值扩展到了可运行的耦合瞬态模型，但还不能消除真实平台的推力、阻力、惯量与绳材辨识缺口。下一步应优先用具体硬件替换假设，再把机体姿态和单旋翼分配加入；对于高刚度且允许松绳的结构，应比较约束/混合动力学模型。',
            '',
            '载重、绳角、风、时延和控制器都会改变表现。相同参考速度下既可能满足也可能不满足研究阈值，因此不应从某个成功工况得出通用安全速度。共点系统不能通过更大增益获得本来缺失的独立姿态力矩。',
            '',
            '没有实现NMPC，也没有以当前两个示范控制器的结果替代原始论文对几何控制、NMPC、INDI或自适应/滑模的应用证据。算法调查与本次自写模型试验分别保留证据层级。','',
            '# 五、复现与文件','',
            '- 模型与队列：simulation/coupled_model.py、run_batch.py；版本快照和SHA在RUN_MANIFEST.json。',
            '- 数值核查：simulation/verify_model.py及results/model_verification.json。',
            '- 完整标量结果：results/cases/*.json与experiment_summary.csv；未完成工况显式标记。',
            '- 日志：WORK_LOG.md、events.jsonl、batch_stdout.log与EXPERIMENT_STATUS.json。',
            '- 原始资料全文核查副本仅在工作目录sources内保存；资料包提供原始链接，不重新分发第三方整篇论文。',
            '- 最终资料包包含全量标量记录、全部已保存批量时序和既有探索时序；完整时序不等于完整系统状态，具体列定义见各NPZ。','',
            '复现需要Python、numpy、matplotlib、scipy（用于其他分析时）；导出另需Pandoc、XeLaTeX、python-docx与报告所用CJK字体。运行批量队列前应先运行verify_model.py。不要不加截止限制地重复启动多个批量进程。']
    md=ROOT/(name+'.md');md.write_text('\n'.join(lines))
    if args.export:
        commands=[
          ['pandoc',str(md),'-o',str(ROOT/(name+'.pdf')),'--pdf-engine=xelatex','--toc','--toc-depth=2',
           '--resource-path='+str(ROOT),'-V','mainfont=Noto Serif CJK SC','-V','sansfont=Noto Sans CJK SC',
           '-V','monofont=DejaVu Sans Mono','-V','fontsize=10pt','-V','geometry=margin=18mm','-V','papersize=a4',
           '-M','lang=en','-H',str(ORIGINAL/'pdf_header.tex')],
          ['pandoc',str(md),'-o',str(ROOT/(name+'.docx')),'--toc','--toc-depth=2','--resource-path='+str(ROOT),
           '--reference-doc='+str(ORIGINAL/'reference.docx')]
        ]
        for command in commands:
            r=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=90)
            (ROOT/(name+'.export.log')).write_text(r.stdout+'\n'+r.stderr)
            if r.returncode:raise RuntimeError(r.stderr[-3000:])
        from assemble_report import assemble
        assemble(ROOT/(name+'.pdf'))
    if final:
        chosen=set()
        for r in baselines+worst:
            if r.get('trace'):chosen.add(ROOT/r['trace'])
        own=[]
        for pattern in ['simulation/*.py','reports/*.md','results/*.json','results/*.csv','results/*.jsonl','results/*.log',
                        'figures/night_*.png','sources/download_manifest*.json','sources/checkpoint_*_downloads.json']:
            own.extend(ROOT.glob(pattern))
        own.extend((ROOT/'results'/'cases').glob('*.json'))
        own.extend((ROOT/'results'/'traces').glob('*.npz'))
        own.extend((ROOT/'results').glob('exploratory*.npz'))
        own.extend(chosen)
        own.extend([ROOT/'WORK_LOG.md',ROOT/'RESEARCH_PLAN.md',ROOT/'STATUS.json',ROOT/'README.md',
                    ROOT/'FINAL_SUMMARY.md',ROOT/'requirements.txt',md,ROOT/'comprehensive_report.pdf'])
        for suffix in ['.pdf','.docx']:
            file=ROOT/(name+suffix)
            if file.exists():own.append(file)
        archive=ROOT.parent/'多架无人机联合吊运_夜间研究成果.zip'
        with ZipFile(archive,'w',ZIP_DEFLATED) as z:
            for file in sorted(set(own)):
                if file.exists():z.write(file,'night_research/'+str(file.relative_to(ROOT)))
            for file in ['report.pdf','report.docx','report.md','report.html','baseline_B0.json','evidence_register.csv',
                         'calculate.py','calculations.json','hover_sweep.csv','build.py','report.css','pdf_header.tex',
                         'reference.docx','requirements.txt','verification.md','checksums.json','README.md']:
                z.write(ORIGINAL/file,'original_v1/'+file)
            for file in (ORIGINAL/'figures').glob('*'):
                if file.is_file():z.write(file,'original_v1/figures/'+file.name)
            z.write(ORIGINAL/'sources'/'download_manifest.json','original_v1/sources/download_manifest.json')
        with ZipFile(archive) as z:assert z.testzip() is None
        print('FINAL_ARCHIVE',archive,archive.stat().st_size)
    print(json.dumps({'generated_utc':now.isoformat(),'final':final,'records':len(current),'complete':len(complete),'report':str(md)},ensure_ascii=False))

if __name__=='__main__':main()
