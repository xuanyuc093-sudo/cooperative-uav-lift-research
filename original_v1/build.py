"""Build report exports and an evidence register; uses pandoc and XeLaTeX."""
from pathlib import Path
import subprocess
import csv
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

ROOT=Path(__file__).resolve().parent
doc=Document()
sec=doc.sections[0]
sec.page_width=Cm(21);sec.page_height=Cm(29.7)
sec.top_margin=Cm(2);sec.bottom_margin=Cm(2)
sec.left_margin=Cm(1.8);sec.right_margin=Cm(1.8)
for name in ['Normal','Body Text','First Paragraph','Table','Caption']:
 try:st=doc.styles[name]
 except KeyError:continue
 st.font.name='Calibri';st.font.size=Pt(10)
 st.element.get_or_add_rPr().append(OxmlElement('w:rFonts'))
 st.element.rPr.rFonts.set(qn('w:eastAsia'),'Noto Serif CJK SC')
 st.paragraph_format.space_after=Pt(6)
 st.paragraph_format.line_spacing=1.15
for name in ['Title','Subtitle','Heading 1','Heading 2','Heading 3']:
 st=doc.styles[name];st.font.name='Calibri';st.font.color.rgb=RGBColor.from_string('173F58')
 st.element.get_or_add_rPr().append(OxmlElement('w:rFonts'))
 st.element.rPr.rFonts.set(qn('w:eastAsia'),'Noto Sans CJK SC')
p=sec.footer.paragraphs[0];p.alignment=2
run=p.add_run('多机联合吊运调查 · ');run.font.size=Pt(8)
field=OxmlElement('w:fldSimple');field.set(qn('w:instr'),'PAGE');p._p.append(field)
doc.save(ROOT/'reference.docx')
def run(args):
 r=subprocess.run(args,cwd=ROOT,text=True,capture_output=True,timeout=50)
 (ROOT/(args[0].split('/')[-1]+'.build.log')).write_text(r.stdout+'\n'+r.stderr)
 if r.returncode:raise RuntimeError(r.stderr[-3000:])
 if r.stderr:print(r.stderr[:3000])
run(['pandoc','report.md','-o','report.docx','--reference-doc=reference.docx','--toc','--toc-depth=2','--resource-path=.'])
doc=Document(ROOT/'report.docx')
for table in doc.tables:
 table.autofit=True
 for idx,row in enumerate(table.rows):
  for cell in row.cells:
   for p in cell.paragraphs:
    for r in p.runs:r.font.size=Pt(8)
  if idx==0:
   hdr=OxmlElement('w:tblHeader');row._tr.get_or_add_trPr().append(hdr)
doc.save(ROOT/'report.docx')
run(['pandoc','report.md','-o','report.html','--standalone','--embed-resources','--mathml','--toc','--toc-depth=2','--css=report.css','--resource-path=.'])
run(['pandoc','report.md','-o','report.pdf','--pdf-engine=xelatex','--toc','--toc-depth=2',
     '--resource-path=.','-V','mainfont=Noto Serif CJK SC','-V','sansfont=Noto Sans CJK SC',
     '-V','monofont=DejaVu Sans Mono','-V','fontsize=10pt','-V','geometry=margin=18mm','-V','papersize=a4',
     '-M','lang=en','-H','pdf_header.tex'])

records=[
 ('FC30_mass','含双电池65 kg，载荷30 kg，最大起飞95 kg','E1','R1','Aircraft','https://www.dji.com/flycart-30/specs','单机产品；不能证明联合吊运能力'),
 ('FC30_speed','20 m/s，海平面无风30 kg载荷','E1','R1','Max Horizontal Speed','https://www.dji.com/flycart-30/specs','最高性能非多机安全速度'),
 ('FC30_brake','S模式无风最小制动距离50 m','E1','R2','printed p.61','https://dl.djicdn.com/downloads/DJI_FlyCart_30/20250709/DJI_FlyCart_30_User_Manual_v1.1_0709.pdf','适用模式与手册条件必须保留'),
 ('FC30_winch','双电池5–30 kg，单电池最高40 kg','E1','R3','Cargo Mounting Methods','https://dl.djicdn.com/downloads/DJI_FlyCart_30/20240110/DJI_FlyCart_30_Winch_System_User_Guide.pdf','单机吊运；控制律未公开'),
 ('ALTA_mass','MTOW34.9 kg，载荷15.9 kg，空重10.4 kg','E1','R4','Weight','https://docs.freeflysystems.com/products/products/alta-x/untitled-3/technical-specs','空重不直接作为含电池质量'),
 ('ALTA_twr','0/10/20/30 lb载荷对应3.5/2.8/2.4/2.0','E1','R5','Thrust-to-weight ratio','https://docs.freeflysystems.com/products/products/alta-x/untitled-3/performance-specs','非持续推力曲线'),
 ('ALTA_conflict','35 lb载荷行总重35.51 kg超过技术页34.9 kg','C','R4,R5','Performance table vs Weight','https://docs.freeflysystems.com/products/products/alta-x/untitled-3/performance-specs','原因未确认；不采用该行推荐初值'),
 ('CF_mass_thrust','34g带支腿；40g载荷；每电机最高30gf','E1','R7','Mechanical/Flight specifications','https://www.bitcraze.io/products/crazyflie-2-1-brushless/','持续推力和速度包线未给出'),
 ('FC100_mass','55.2/60.2kg明确排除电池等项','E1','R8','Weight footnote','https://www.dji.com/flycart-100/specs','不能作为含电池飞行质量'),
 ('RSS2013','每机0.5kg；三机；载荷0.225kg；绳1m','E2-experiment','R9','Table III, IV-B','https://roboticsproceedings.org/rss09/p11.pdf','完整复现还需惯量挂点和控制参数'),
 ('LEE2014','每机0.755kg；三机；载荷1.5kg；绳1m','E2-simulation','R10','Section V','https://arxiv.org/abs/1403.3684','不是实体实验'),
 ('SUN2025_scale','每机0.6kg；三机；载荷1.4kg；绳1m','E2-experiment','R11','Experimental Setup','https://arxiv.org/html/2501.18802v2','B0只参考尺度，不是整机复刻'),
 ('SUN2025_fast','Fast参考5m/s、8m/s²','E2-experiment','R11','Table 1, Fig.2','https://arxiv.org/html/2501.18802v2','特定实验工况；不是通用安全速度'),
 ('B0_hover','每机推力7.880N；绳张力2.088N','C','Equations 8–10','calculate.py','report.md','仅对称无风静力；未闭环验证'),
 ('B0_inputs','16N可用、12.8N规划、1m/s初速上限','A','Section 6','baseline_B0.json','baseline_B0.json','必须辨识和闭环验证'),
 ('B0_rank','G秩6；固定绳向A秩3','C','Equations 21–22','calculate.py','calculate.py','几何映射不等于瞬时全驱动'),
 ('QP_geometric','最多三机的机载微控制器实体系统','E2-experiment','R13','Abstract, Experiments','https://arxiv.org/html/2304.02359v2','约束与模型假设必须核查'),
 ('NMPC','六自由度载荷操纵，仿真及实体验证','E2','R14','Original abstract','https://arxiv.org/abs/2303.06165','本报告未提取该文具体速度'),
 ('RISE','两机棒状载荷实验，3.5m/s²工况','E2-abstract','R18','Abstract','https://link.springer.com/article/10.1007/s10846-024-02174-4','不是完整速度包线'),
 ('Hinf','Fly-Crane不确定性鲁棒设计及实物实验','E2-abstract','R19','Author institutional abstract','https://iris.uniroma1.it/handle/11573/1688362','多绳结构不能直接等同一机一绳'),
 ('AutoMultilift','学习MPC代价参数，所读v5仿真','E2-simulation','R20','Abstract','https://arxiv.org/abs/2406.04858','未声明实体验证'),
 ('RotorTM','混合绳索动态仿真器与实体对比','E2','R21','Abstract and author repository','https://arxiv.org/abs/2205.05140','本轮未安装运行'),
 ('Y_topology','Y形相对V形增加摆动建模复杂度','E2','R23','Original paper abstract/introduction','https://www.mdpi.com/2504-446X/8/3/103','未作为实飞速度证据'),
 ('SMC2026','最多五机实体试验及编队重构','E2-abstract','R24','Original abstract','https://pubmed.ncbi.nlm.nih.gov/41781298/','本轮未读全文参数，不引用性能排名')
]
with (ROOT/'evidence_register.csv').open('w',newline='',encoding='utf-8-sig') as f:
 w=csv.writer(f);w.writerow(['claim_id','claim','evidence','reference','location','url','limitation']);w.writerows(records)
print('Exports created: report.docx, report.html, report.pdf; evidence rows:',len(records))
