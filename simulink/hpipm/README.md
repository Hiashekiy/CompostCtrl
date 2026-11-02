# HPIPM 原生迁移接口

构建：MATLAB 中 `addpath('simulink/hpipm'); build_hpipm_mex`。
测试：先运行 `python simulink/py/export_hpipm_qp.py`，再在 MATLAB 执行 `check_hpipm_snapshot; build_hpipm_interface`。

运行依赖本机 `D:/Tools/hpipm` 与 `D:/Tools/blasfeo`；启动执行 `hpipm_startup`。
`mex/` 中已生成 Windows x64 二进制；跨平台或更换 ABI 时须重新构建。
官方 MATLAB wrapper/C 版权与 BSD 许可仍归 HPIPM 作者，脚本直接编译本机原文件，没有将第三方源码拷入项目。

详细接口契约、误差报告、编译警告及尚未实现部分见 `docs/simulink/07_hpipm_matlab_migration.md`。
