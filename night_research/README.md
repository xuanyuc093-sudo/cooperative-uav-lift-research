# 多架无人机联合吊运夜间成果

研究截止：2026-11-01 06:00 UTC+8。620条模型记录已关闭；截止后的修改仅为汇总、导出和交付核验。

## 阅读与证据

- `comprehensive_report.pdf`：原始17页报告与最终夜间补充报告的合订本，附分部书签。
- `overnight_report.pdf/.docx/.md`：最终夜间报告，包含截止总结及四个专题。
- `FINAL_SUMMARY.md`：最终数量、证据分类、结论和未完成内容。
- `reports/`：四个专题；专题四的499条记录是固定检查点，不是最终620条统计。
- `results/FINAL_DATA_AUDIT.json`：620条记录与612份批量时序的一致性核查。

厂商数据、论文实飞、论文仿真/理论、本轮简化模型、解析计算和未验证假设分别标注。没有真机安全认证；假设上限不是持续或峰值额定推力。

## 资料包目录

解压后`original_v1/`保留原始报告、参数、计算脚本、CSV/JSON和图；`night_research/`包含本目录内容。出版脚本同时支持原工作目录的`uav_lift_report/`和解压包的`original_v1/`，无需固定安装在`/workspace`。

`results/cases/*.json`是620条原子提交标量记录；`results/traces/*.npz`是612份批量时序。初始不可行的8条没有时间积分；截断或提前终止时序保持原样。NPZ的`columns`定义19列，不包含全部40维状态、单旋翼指令或绳方向。另保留`results/exploratory*.npz`的既有探索记录。

## 复核和复现

核心Python版本及库版本见`results/checkpoint_04_runtime.json`和`requirements.txt`。PDF导出另需Pandoc、XeLaTeX、Noto CJK字体；已有最终报告可直接阅读。

在`night_research/`目录下，仅做已有文件核验可执行：

```bash
python simulation/final_delivery_audit.py
```

该命令不积分动力学；它检查原报告校验和、记录/状态/CSV、标签和已保存时序。

专题四分析可由`python simulation/analyze_completed_snapshot.py`读取固定499条JSON快照复现。静力、分配、制动、冲量脚本会重算其独立算例，并不等于实飞。部分较早审查脚本读取当前case目录，重新运行会得到不同于早期快照的统计；原始结果与时间戳已保存。

本次截止后没有重启`run_batch.py`或执行新的动力学验证。它是具有截止参数的实验入口，需由后续研究明确设定新任务后使用，不是查看报告所必需的步骤。原始`baseline_B0.json`不是PX4/ArduPilot即插即用配置；夜间运行参数以各case的`config`和`RUN_MANIFEST.json`为准。

## 完整性

`original_v1/checksums.json`核对原始材料；`RUN_MANIFEST.json`记录模型和启动队列快照；`FINAL_TRACE_REGISTER.csv`记录全部批量时序SHA；`FINAL_DATA_AUDIT.json`保存各case SHA。最终ZIP另提供SHA-256文件。不要将文件一致性通过解释为物理模型正确性证明。
