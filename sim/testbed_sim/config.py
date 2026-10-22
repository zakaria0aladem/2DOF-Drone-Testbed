"""Load default_config.toml and merge a user file on top of it."""
from __future__ import annotations

import copy
from pathlib import Path

try:
    import tomllib
except ImportError:                     # Python < 3.11
    import tomli as tomllib             # pip install tomli

DEFAULT = Path(__file__).with_name("default_config.toml")


def _merge(base, over, path=""):
    for k, v in over.items():
        if k not in base:
            raise KeyError(f"unknown config key '{path}{k}' (check the spelling against "
                           f"default_config.toml)")
        if isinstance(v, dict) and isinstance(base[k], dict):
            _merge(base[k], v, f"{path}{k}.")
        else:
            base[k] = v


def load(user=None, overrides=None):
    """user: path to a .toml file, or a dict. overrides: dict merged last."""
    with open(DEFAULT, "rb") as f:
        cfg = tomllib.load(f)
    if user is not None:
        if isinstance(user, dict):
            _merge(cfg, copy.deepcopy(user))
        else:
            with open(user, "rb") as f:
                _merge(cfg, tomllib.load(f))
    if overrides:
        _merge(cfg, copy.deepcopy(overrides))
    return cfg
