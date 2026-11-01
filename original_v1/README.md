# 多架无人机联合吊运研究资料包

核验日期：2026-10-31。

先阅读 `report.pdf`；需要编辑时使用 `report.docx` 或 `report.md`。报告覆盖主要任务与进阶任务，包含24项参考资料、参数证据、受力推导、初值建议、算法及挂点拓扑比较。

`baseline_B0.json` 是研究初值及来源标记，不是某一仿真器的即插即用配置。`calculate.py` 只复现静力与速度约束的解析筛查，不包含闭环控制器，不构成实飞安全速度验证。

复算：

```bash
python calculate.py
```

需要 Python、numpy、matplotlib。输出 `calculations.json`、`hover_sweep.csv` 和三张原始图。

重新导出报告：

```bash
python build.py
```

另需 python-docx、Pandoc、XeLaTeX、Noto Serif CJK SC / Noto Sans CJK SC / DejaVu Sans Mono字体。构建脚本会生成Word样式基准、Word、可离线阅读的HTML、PDF及证据表。

`evidence_register.csv` 为UTF-8 BOM编码，可用常用表格软件打开。E1表示厂商原始数据，E2表示论文数据（区分摘要、仿真和实体实验），C表示计算，A表示待验证假设。

资料包不包含第三方论文、厂商手册的完整副本；原始链接在报告和证据表中。工作目录中的下载文件只用于本轮核验。

本轮已经完成文献与规格核查、静力一致性计算、图表和公式导出检查。尚未实现或运行完整多体闭环仿真，未进行真机辨识与实飞试验。目标机型和任务环境明确后，应优先替换假设的推力、惯量、阻力、时延及绳材参数。
