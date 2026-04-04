"""Parametrized check: sum of `size` in deploy.yaml observations matches policy ONNX input."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_DEPLOY_ROOT = Path(__file__).resolve().parents[1]
_SCRIPT = _DEPLOY_ROOT / "scripts" / "check_deploy_onnx_observation.py"

_spec = importlib.util.spec_from_file_location(
    "check_deploy_onnx_observation", _SCRIPT
)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
sys.modules["check_deploy_onnx_observation"] = _mod
_spec.loader.exec_module(_mod)

check_deploy_onnx_observation = _mod.check_deploy_onnx_observation


def _deploy_onnx_pairs():
    pairs: list[tuple[Path, Path]] = []
    for deploy in sorted(_DEPLOY_ROOT.glob("robots/**/params/deploy.yaml")):
        onnx = deploy.parent.parent / "exported" / "policy.onnx"
        if onnx.is_file():
            pairs.append((deploy, onnx))
    return pairs


_PAIRS = _deploy_onnx_pairs()
_PAIR_IDS = [str(d.relative_to(_DEPLOY_ROOT)) for d, _ in _PAIRS]


@pytest.mark.parametrize("deploy_yaml,policy_onnx", _PAIRS, ids=_PAIR_IDS)
def test_declared_observation_sizes_match_onnx(
    deploy_yaml: Path, policy_onnx: Path
) -> None:
    check_deploy_onnx_observation(deploy_yaml, policy_onnx)
