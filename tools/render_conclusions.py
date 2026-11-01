"""Export the conclusion supplement and prepend it to the unchanged report."""
from pathlib import Path
import json
import subprocess
import fitz

ROOT=Path(__file__).resolve().parents[1]


def main():
    common=['pandoc',str(ROOT/'CONCLUSIONS.md'),'--toc','--toc-depth=2','--resource-path='+str(ROOT)]
    pdf=common+['-o',str(ROOT/'CONCLUSIONS.pdf'),'--pdf-engine=xelatex',
        '-V','mainfont=Noto Serif CJK SC','-V','sansfont=Noto Sans CJK SC',
        '-V','monofont=DejaVu Sans Mono','-V','fontsize=10pt','-V','geometry=margin=18mm',
        '-V','papersize=a4','-M','lang=en','-H',str(ROOT/'original_v1/pdf_header.tex')]
    docx=common+['-o',str(ROOT/'CONCLUSIONS.docx'),'--reference-doc='+str(ROOT/'original_v1/reference.docx')]
    for command in [pdf,docx]:
        r=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=90)
        if r.returncode:raise RuntimeError(r.stderr[-3000:])
    combined=fitz.open();toc=[];parts=[];flags=[]
    for label,path in [('研究结论与实现建议',ROOT/'CONCLUSIONS.pdf'),
                       ('原报告与夜间研究（保留原文）',ROOT/'night_research/comprehensive_report.pdf')]:
        with fitz.open(path) as source:
            offset=len(combined);toc.append([1,label,offset+1])
            for level,title,page in source.get_toc():
                if page>0:toc.append([level+1,title,page+offset])
            combined.insert_pdf(source);parts.append({'file':str(path.relative_to(ROOT)),'pages':len(source)})
            if path.name=='CONCLUSIONS.pdf':
                for i,page in enumerate(source):
                    for block in page.get_text('dict')['blocks']:
                        for line in block.get('lines',[]):
                            for span in line['spans']:
                                if span['bbox'][0]<29 or span['bbox'][2]>page.rect.width-29:
                                    flags.append({'page':i+1,'text':span['text']})
    combined.set_toc(toc)
    combined.set_metadata({'title':'多架无人机联合吊运：具体结论、原始调查与夜间研究'})
    combined.save(ROOT/'report_with_conclusions.pdf',garbage=4,deflate=True)
    result={'conclusion_pages':parts[0]['pages'],'combined_pages':len(combined),
            'parts':parts,'horizontal_boundary_flags':flags,'original_report_replaced':False}
    (ROOT/'CONCLUSION_EXPORT_REVIEW.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
