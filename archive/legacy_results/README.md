# legacy_results

本目录**不存放数据副本**。按归档策略，`results/`（原始时序/指标/元数据）与
`figures/`（图件）只登记在 `../ARCHIVE_MANIFEST.csv` 中，逐文件记录
原始路径、字节数与 SHA-256，并注明其所在 git 提交，避免同一仓库内
重复存储约 50 MB 历史数据。

取回方式：

```bash
git checkout 6fcc0f9 -- results figures      # 按提交号恢复
python tools/archive_legacy.py verify        # 校核当前工作区哈希
```

子目录 `csv/ figures/ logs/ metrics/` 保留为空骨架，用于将来确实需要
物理副本时（`python tools/archive_legacy.py build --include-results`）。
