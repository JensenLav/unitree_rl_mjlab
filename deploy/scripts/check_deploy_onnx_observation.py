#!/usr/bin/env python3
"""Verify deploy.yaml matches policy.onnx (observations, joint_ids_map, actions).

Checks:
  - len(joint_ids_map) == joint_pos_rel.size == joint_vel_rel.size (when those `size` keys exist).
    Shrinking joint_ids_map to e.g. 0–11 while joint obs still declare 27 fails here.
  - Sum of observation `size` fields == ONNX input `obs` element count.
  - Declared action dim (action `scale` length, or `joint_ids` length) == ONNX `actions` output.

Usage:
  python deploy/scripts/check_deploy_onnx_observation.py \\
    deploy/robots/h1_2/config/policy/velocity/v0/params/deploy.yaml \\
    deploy/robots/h1_2/config/policy/velocity/v0/exported/policy.onnx

Options:
  --input-name NAME   ONNX input tensor to check (default: obs)
  --obs-group NAME    YAML observations subgroup (e.g. actor) when not flat layout
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

try:
    import onnx
except ImportError as e:  # pragma: no cover
    print("check_deploy_onnx_observation: install onnx (pip install onnx)", file=sys.stderr)
    raise SystemExit(2) from e

try:
    import yaml
except ImportError as e:  # pragma: no cover
    print("check_deploy_onnx_observation: install pyyaml", file=sys.stderr)
    raise SystemExit(2) from e


def _iter_term_entries(term_map: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    out: list[tuple[str, dict[str, Any]]] = []
    for key, val in term_map.items():
        if key in ("scale_first", "use_gym_history"):
            continue
        if isinstance(val, dict) and "params" in val:
            out.append((key, val))
    return out


def observation_terms_from_deploy(
    observations: dict[str, Any],
    obs_group: str | None,
) -> list[tuple[str, dict[str, Any]]]:
    """Match deploy/include/isaaclab/manager/observation_manager.h layout."""
    if not isinstance(observations, dict):
        raise ValueError("'observations' must be a mapping")

    top = {
        k: v
        for k, v in observations.items()
        if k not in ("scale_first", "use_gym_history")
    }
    if not top:
        return []

    _, first_v = next(iter(top.items()))
    if isinstance(first_v, dict) and "params" in first_v:
        return _iter_term_entries(top)

    # Grouped: actor / critic / ...
    group = obs_group
    if group is None:
        group = next(iter(top.keys()))
    if group not in top:
        raise KeyError(f"observations group '{group}' not found; have: {list(top.keys())}")
    return _iter_term_entries(top[group])


def sum_declared_obs_sizes(
    observations: dict[str, Any],
    obs_group: str | None = None,
) -> tuple[int, list[tuple[str, int | None]]]:
    """Returns (total, [(term, size or None if missing), ...])."""
    terms = observation_terms_from_deploy(observations, obs_group)
    if not terms:
        return 0, []

    details: list[tuple[str, int | None]] = []
    total = 0
    for name, cfg in terms:
        if "size" not in cfg or cfg["size"] is None:
            details.append((name, None))
            continue
        sz = int(cfg["size"])
        details.append((name, sz))
        total += sz
    return total, details


def onnx_input_element_count(
    model_path: Path,
    input_name: str,
) -> tuple[int | None, list[tuple[str, list[int | str]]]]:
    """Returns (element_count or None if dynamic, [(name, shape_dims), ...])."""
    model = onnx.load(str(model_path))
    inputs_info: list[tuple[str, list[int | str]]] = []
    target_shape: list[int | str] | None = None
    for inp in model.graph.input:
        t = inp.type.tensor_type
        shape: list[int | str] = []
        has_dynamic = False
        for d in t.shape.dim:
            if d.dim_value:
                shape.append(int(d.dim_value))
            elif d.dim_param:
                shape.append(d.dim_param)
                has_dynamic = True
            else:
                has_dynamic = True
                shape.append("?")
        inputs_info.append((inp.name, shape))
        if inp.name == input_name:
            target_shape = shape

    if target_shape is None:
        names = [n for n, _ in inputs_info]
        raise KeyError(
            f"ONNX input '{input_name}' not found. Inputs: {names}"
        )

    n = 1
    for dim in target_shape:
        if isinstance(dim, str):
            return None, inputs_info
        n *= int(dim)
    return n, inputs_info


def onnx_output_element_count(
    model_path: Path,
    output_name: str = "actions",
) -> tuple[int | None, list[tuple[str, list[int | str]]]]:
    """Returns (element_count or None if dynamic, [(name, shape_dims), ...])."""
    model = onnx.load(str(model_path))
    outs_info: list[tuple[str, list[int | str]]] = []
    target_shape: list[int | str] | None = None
    for out in model.graph.output:
        t = out.type.tensor_type
        shape: list[int | str] = []
        for d in t.shape.dim:
            if d.dim_value:
                shape.append(int(d.dim_value))
            elif d.dim_param:
                shape.append(d.dim_param)
            else:
                shape.append("?")
        outs_info.append((out.name, shape))
        if out.name == output_name:
            target_shape = shape

    if target_shape is None and len(model.graph.output) == 1:
        target_shape = outs_info[0][1]

    if target_shape is None:
        names = [n for n, _ in outs_info]
        raise KeyError(
            f"ONNX output '{output_name}' not found. Outputs: {names}"
        )

    n = 1
    for dim in target_shape:
        if isinstance(dim, str):
            return None, outs_info
        n *= int(dim)
    return n, outs_info


def validate_joint_ids_map_matches_joint_observation_sizes(
    data: dict[str, Any],
    deploy_yaml: Path,
    obs_group: str | None,
) -> None:
    """joint_ids_map length must match joint_pos_rel / joint_vel_rel `size` (deploy stack contract)."""
    jm = data.get("joint_ids_map")
    if not isinstance(jm, list):
        return
    n = len(jm)
    obs = data.get("observations")
    if not isinstance(obs, dict):
        return
    terms = dict(observation_terms_from_deploy(obs, obs_group))
    for key in ("joint_pos_rel", "joint_vel_rel", "joint_pos", "joint_vel"):
        if key not in terms:
            continue
        cfg = terms[key]
        if "size" not in cfg or cfg["size"] is None:
            continue
        sz = int(cfg["size"])
        if sz != n:
            raise AssertionError(
                f"{deploy_yaml}: len(joint_ids_map) is {n} but observation '{key}' "
                f"declares size {sz}. These must match: state is read only for motors "
                f"listed in joint_ids_map (same order as joint_pos / joint_vel observations)."
            )


def declared_policy_action_dim(data: dict[str, Any]) -> int | None:
    """Action vector length implied by deploy.yaml (scale list or joint_ids)."""
    actions = data.get("actions")
    if not isinstance(actions, dict):
        return None
    for term in ("JointPositionAction", "JointVelocityAction"):
        if term not in actions:
            continue
        cfg = actions[term]
        jids = cfg.get("joint_ids")
        if jids is not None and isinstance(jids, list) and len(jids) > 0:
            return len(jids)
        sc = cfg.get("scale")
        if isinstance(sc, list) and len(sc) > 0:
            return len(sc)
        jm = data.get("joint_ids_map")
        if isinstance(jm, list):
            return len(jm)
        return None
    return None


def validate_action_dim_matches_onnx_output(
    data: dict[str, Any],
    deploy_yaml: Path,
    onnx_path: Path,
    *,
    output_name: str = "actions",
) -> None:
    expected = declared_policy_action_dim(data)
    if expected is None:
        return
    onnx_n, outs = onnx_output_element_count(onnx_path, output_name)
    if onnx_n is None:
        raise AssertionError(
            f"{onnx_path}: output '{output_name}' has dynamic shape; cannot verify action size.\n"
            f"  outputs: {outs}"
        )
    if expected != onnx_n:
        raise AssertionError(
            f"Policy action dimension mismatch.\n"
            f"  deploy: {deploy_yaml}\n"
            f"  onnx:   {onnx_path}\n"
            f"  deploy implies action dim {expected} (from action scale / joint_ids)\n"
            f"  ONNX output '{output_name}' has {onnx_n} elements\n"
            f"  ONNX outputs: {outs}"
        )


def check_deploy_onnx_observation(
    deploy_yaml: Path,
    onnx_path: Path,
    *,
    input_name: str = "obs",
    obs_group: str | None = None,
) -> None:
    """Raises AssertionError with a message if sizes do not match."""
    data = yaml.safe_load(deploy_yaml.read_text())
    if "observations" not in data:
        raise AssertionError(f"{deploy_yaml}: missing top-level 'observations'")

    validate_joint_ids_map_matches_joint_observation_sizes(
        data, deploy_yaml, obs_group
    )

    total, details = sum_declared_obs_sizes(data["observations"], obs_group)
    missing = [n for n, s in details if s is None]
    if missing:
        raise AssertionError(
            f"{deploy_yaml}: observation term(s) missing 'size': {missing}. "
            "Add `size: N` under each observation term to enable this check."
        )

    onnx_elems, all_inputs = onnx_input_element_count(onnx_path, input_name)
    if onnx_elems is None:
        shapes = "\n".join(f"  {n}: {s}" for n, s in all_inputs)
        raise AssertionError(
            f"{onnx_path}: input '{input_name}' has dynamic/unknown shape; "
            f"cannot verify statically.\n{shapes}"
        )

    if total != onnx_elems:
        breakdown = ", ".join(f"{n}={s}" for n, s in details)
        raise AssertionError(
            f"Observation size mismatch.\n"
            f"  deploy: {deploy_yaml}\n"
            f"  onnx:   {onnx_path}\n"
            f"  input:  {input_name}\n"
            f"  declared observation sizes sum: {total} ({breakdown})\n"
            f"  ONNX tensor elements:           {onnx_elems}\n"
            f"  ONNX inputs: {all_inputs}"
        )

    validate_action_dim_matches_onnx_output(data, deploy_yaml, onnx_path)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("deploy_yaml", type=Path, help="Path to params/deploy.yaml")
    p.add_argument("policy_onnx", type=Path, help="Path to exported policy.onnx")
    p.add_argument(
        "--input-name",
        default="obs",
        help="ONNX input tensor name (default: obs)",
    )
    p.add_argument(
        "--obs-group",
        default=None,
        help="Observations subgroup key when YAML uses grouped layout",
    )
    args = p.parse_args(argv)

    deploy = args.deploy_yaml.resolve()
    onnx_f = args.policy_onnx.resolve()
    if not deploy.is_file():
        print(f"Not found: {deploy}", file=sys.stderr)
        return 1
    if not onnx_f.is_file():
        print(f"Not found: {onnx_f}", file=sys.stderr)
        return 1

    try:
        check_deploy_onnx_observation(
            deploy,
            onnx_f,
            input_name=args.input_name,
            obs_group=args.obs_group,
        )
    except (AssertionError, KeyError, ValueError) as e:
        print(e, file=sys.stderr)
        return 1

    loaded = yaml.safe_load(deploy.read_text())
    total, details = sum_declared_obs_sizes(loaded["observations"], args.obs_group)
    print(f"OK: observation sizes sum to {total} (matches ONNX input '{args.input_name}')")
    for name, sz in details:
        print(f"  {name}: {sz}")
    jm = loaded.get("joint_ids_map")
    if isinstance(jm, list):
        print(f"OK: len(joint_ids_map) == {len(jm)} (matches joint_pos/vel observation sizes)")
    ad = declared_policy_action_dim(loaded)
    if ad is not None:
        print(f"OK: policy action dim {ad} (matches ONNX output 'actions')")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
