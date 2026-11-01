"""Join the unchanged original report and the current supplement for reading."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import fitz

ROOT = Path(__file__).resolve().parents[1]
ORIGINAL = ROOT.parent/'uav_lift_report'/'report.pdf'
if not ORIGINAL.exists():ORIGINAL=ROOT.parent/'original_v1'/'report.pdf'


def assemble(supplement):
    supplement = Path(supplement).resolve()
    output = ROOT/'comprehensive_report.pdf'
    toc, parts = [], []
    combined = fitz.open()
    for label, path in [('原始调查报告 v1（保留原文）', ORIGINAL),
                        ('夜间补充研究（以其封面时间为准）', supplement)]:
        with fitz.open(path) as source:
            offset = len(combined)
            toc.append([1, label, offset+1])
            for level, title, page in source.get_toc():
                if page > 0:
                    toc.append([level+1, title, page+offset])
            combined.insert_pdf(source)
            parts.append(dict(path=str(path), page_count=len(source),
                              starts_on_page=offset+1,
                              sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    combined.set_toc(toc)
    combined.set_metadata({'title':'多架无人机联合吊运：原始调查与夜间补充研究',
                           'subject':'原报告与补充研究的阅读合订本；非实飞安全认证'})
    temp = output.with_name(output.stem+'.'+str(os.getpid())+'.tmp.pdf')
    combined.save(temp, garbage=4, deflate=True)
    pages = len(combined)
    combined.close()
    with fitz.open(temp) as check:
        assert len(check) == sum(p['page_count'] for p in parts)
        assert len(check.get_toc()) >= 2
    temp.replace(output)
    manifest = dict(generated_utc=datetime.now(timezone.utc).isoformat(),
                    output=str(output), pages=pages, parts=parts,
                    interpretation='Reading assembly only; original PDF unchanged. Supplement distinguishes evidence, assumptions and simulations.')
    (ROOT/'results'/'assembly_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--supplement',required=True)
    args=parser.parse_args()
    print(json.dumps(assemble(args.supplement),ensure_ascii=False))
