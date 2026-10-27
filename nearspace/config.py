"""Configuration loading helpers (YAML -> attribute-access namespace).

An experiment file may start with a `base:` key naming another YAML file
(relative to the experiment file); its content is deep-merged over that base.
That is how every A/B run shares one frozen aircraft / atmosphere / controller
definition and differs only in the actuator block.
"""
from __future__ import annotations

import copy
import os
from types import SimpleNamespace
from typing import Any

import yaml


def _to_ns(value: Any) -> Any:
    if isinstance(value, dict):
        return SimpleNamespace(**{k: _to_ns(v) for k, v in value.items()})
    if isinstance(value, list):
        return [_to_ns(v) for v in value]
    return value


def deep_merge(dst: dict, src: dict) -> dict:
    out = copy.deepcopy(dst)
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


_deep_merge = deep_merge


def read_yaml(path: str, _seen=None) -> dict:
    """Load a YAML file, resolving nested `base:` includes."""
    path = os.path.abspath(path)
    _seen = set() if _seen is None else _seen
    if path in _seen:
        raise ValueError("circular base: include at %s" % path)
    _seen.add(path)
    with open(path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    base = raw.pop("base", None)
    if base:
        base_path = base if os.path.isabs(base) else os.path.join(os.path.dirname(path), base)
        raw = _deep_merge(read_yaml(base_path, _seen), raw)
    return raw


def load_config(path: str) -> SimpleNamespace:
    return _to_ns(read_yaml(path))


def dump(cfg) -> dict:
    """Plain-dict snapshot of an effective configuration (for the run metadata)."""
    if isinstance(cfg, dict):
        return {k: dump(v) for k, v in cfg.items()}
    if isinstance(cfg, (list, tuple)):
        return [dump(v) for v in cfg]
    if isinstance(cfg, SimpleNamespace):
        return {k: dump(v) for k, v in vars(cfg).items()}
    if hasattr(cfg, "tolist"):
        return cfg.tolist()
    return cfg
