"""配置加载（指导实施文件 §12）：一切参数进 YAML，代码里不藏常数。

实验文件可用 `base:` 指向另一个 YAML，其内容深合并覆盖 base（用于 A/B/C/D 共用一套
飞行器/大气/控制器定义，只切换执行机构与搜索策略）。

**强制校验**：`search.max_expansions` 与 `search.max_search_time_ms` 未设置（null 或缺省）时
直接抛异常启动失败，不允许默认无限搜索（指导实施文件 §12）。
"""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

EXP_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = EXP_ROOT / "configs"


def deep_merge(dst: dict, src: dict) -> dict:
    out = copy.deepcopy(dst)
    for k, v in (src or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def read_yaml(path: str | Path, _seen: tuple = ()) -> dict:
    path = Path(path)
    if str(path) in _seen:
        raise ValueError(f"circular base: include: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    base = data.pop("base", None)
    if base:
        parent = read_yaml((path.parent / base).resolve(), _seen + (str(path),))
        data = deep_merge(parent, data)
    return data


class ConfigError(ValueError):
    pass


def validate(cfg: dict) -> None:
    search = cfg.get("search") or {}
    if search.get("max_expansions") is None:
        raise ConfigError("search.max_expansions 未设置：禁止默认无限搜索（见 configs/search.yaml）")
    if search.get("max_search_time_ms") is None:
        raise ConfigError("search.max_search_time_ms 未设置：禁止默认无限搜索（见 configs/search.yaml）")
    for key in ("simulation", "vehicle", "aero", "cmg", "control", "allocation"):
        if key not in cfg:
            raise ConfigError(f"缺少配置段：{key}")


def load_config(*paths: str | Path, validate_it: bool = True) -> dict:
    """按顺序合并若干 YAML（后者覆盖前者）。"""
    cfg: dict = {}
    for p in paths:
        path = Path(p)
        if not path.is_absolute():
            path = (CONFIG_DIR / p) if (CONFIG_DIR / p).exists() else Path.cwd() / p
        cfg = deep_merge(cfg, read_yaml(path))
    if validate_it:
        validate(cfg)
    return cfg
