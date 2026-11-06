# 历史重复原始输出清理

仅清理cleanup_manifest.csv逐项列出的历史原始输出副本；每一份均已完整复制到paper_final，且旧文件、保留文件与已提交的SHA清单三方一致。paper_final是保留归档，不再额外复制一份相同大文件。99份原始保护结果、参考bank、场景配置、模型、源码、数据依赖、未筛选的历史MAT及Git证据不在删除范围内。

历史revision的read_only_snapshot/protected_snapshot是当时验收的原始证据，保持原文件不变。部分旧路径在本次获准清理后迁移至paper_final；旧快照不再表示这些工作区副本仍然存在。以本迁移清单解析保留位置，数据内容与原SHA未变。

先提交清单和验证凭证，再逐文件删除。删除后验证paper_final全部197例原始数据和3303份不可变文件。执行结果见deletion_receipt.json；论文集原有验收凭证保持不变。

需要恢复旧路径时，从仓库根目录运行：

```powershell
python simulink/paper/final/cleanup_history_duplicates.py restore
```

恢复逐文件核对SHA；已有同名文件不覆盖。恢复会重新占用约6.75GB。本次没有删除未在论文集保留的历史负面结果。
