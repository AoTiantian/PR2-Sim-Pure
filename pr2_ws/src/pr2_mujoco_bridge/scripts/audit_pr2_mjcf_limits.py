#!/usr/bin/env python3
"""Audit PR2 MJCF force, damping, and contact limit-like fields.

This helper is intentionally read-only: it parses MJCF XML and reports values that
can limit or hide whole-body admittance motion before any tuning/XML edits are
made.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from typing import Any, Dict, Iterable, List, Optional

LIMIT_ATTRS = (
    "range",
    "actuatorfrcrange",
    "damping",
    "frictionloss",
    "gear",
    "ctrlrange",
    "forcerange",
    "gainprm",
    "biastype",
    "friction",
    "condim",
    "solref",
    "solimp",
    "contype",
    "conaffinity",
)

JOINT_ATTRS = ("name", "type", "range", "actuatorfrcrange", "damping", "frictionloss")
ACTUATOR_ATTRS = ("name", "joint", "gear", "ctrlrange", "forcerange", "gainprm", "biastype")
GEOM_ATTRS = ("name", "type", "friction", "condim", "solref", "solimp", "contype", "conaffinity")

BASE_PATTERNS = ("caster", "wheel", "base")
TORSO_PATTERNS = ("torso",)
LEFT_ARM_PATTERNS = ("l_shoulder", "l_upper_arm", "l_elbow", "l_forearm", "l_wrist")
RIGHT_ARM_PATTERNS = ("r_shoulder", "r_upper_arm", "r_elbow", "r_forearm", "r_wrist")
GRIPPER_PATTERNS = ("gripper", "finger", "tip")
HEAD_PATTERNS = ("head", "laser", "sensor")


def _local_name(tag: str) -> str:
    """Return an XML tag without a namespace prefix."""
    return tag.rsplit("}", 1)[-1]


def _select_attrs(elem: ET.Element, attrs: Iterable[str]) -> Dict[str, str]:
    return {attr: elem.attrib[attr] for attr in attrs if attr in elem.attrib}


def infer_subsystem(name: str, *, kind: str = "") -> str:
    """Map a PR2 joint/actuator/geom name to a coarse subsystem bucket."""
    lowered = name.lower()
    if any(token in lowered for token in BASE_PATTERNS):
        return "base_casters_wheels"
    if any(token in lowered for token in TORSO_PATTERNS):
        return "torso"
    if any(token in lowered for token in LEFT_ARM_PATTERNS):
        return "left_arm"
    if any(token in lowered for token in RIGHT_ARM_PATTERNS):
        return "right_arm"
    if any(token in lowered for token in GRIPPER_PATTERNS):
        return "grippers"
    if any(token in lowered for token in HEAD_PATTERNS):
        return "head_sensors"
    if kind == "geom" and any(token in lowered for token in ("floor", "ground", "terrain")):
        return "scene_contact"
    return "other"


def _entry(elem: ET.Element, attrs: Iterable[str], *, kind: str, source: Path) -> Dict[str, Any]:
    selected = _select_attrs(elem, attrs)
    name = selected.get("name") or selected.get("joint") or "<unnamed>"
    return {
        "kind": kind,
        "tag": _local_name(elem.tag),
        "name": name,
        "subsystem": infer_subsystem(name, kind=kind),
        "source": str(source),
        "attributes": selected,
    }


def parse_mjcf(path: Path) -> Dict[str, List[Dict[str, Any]]]:
    """Parse one MJCF/XML file and return limit-like entries."""
    tree = ET.parse(path)
    root = tree.getroot()
    joints: List[Dict[str, Any]] = []
    actuators: List[Dict[str, Any]] = []
    geoms: List[Dict[str, Any]] = []
    other_limit_fields: List[Dict[str, Any]] = []

    for elem in root.iter():
        tag = _local_name(elem.tag)
        if tag == "joint":
            entry = _entry(elem, JOINT_ATTRS, kind="joint", source=path)
            if len(entry["attributes"]) > 1:
                joints.append(entry)
        elif tag in {"motor", "velocity", "position", "general", "intvelocity", "cylinder"}:
            entry = _entry(elem, ACTUATOR_ATTRS, kind="actuator", source=path)
            if len(entry["attributes"]) > 1:
                actuators.append(entry)
        elif tag == "geom":
            entry = _entry(elem, GEOM_ATTRS, kind="geom", source=path)
            if any(attr in entry["attributes"] for attr in ("friction", "condim", "solref", "solimp", "contype", "conaffinity")):
                geoms.append(entry)
        else:
            selected = _select_attrs(elem, LIMIT_ATTRS)
            if selected:
                name = elem.attrib.get("name", f"<{tag}>")
                other_limit_fields.append(
                    {
                        "kind": "other",
                        "tag": tag,
                        "name": name,
                        "subsystem": infer_subsystem(name),
                        "source": str(path),
                        "attributes": selected,
                    }
                )
    return {
        "joints": joints,
        "actuators": actuators,
        "geoms": geoms,
        "other_limit_fields": other_limit_fields,
    }


def group_entries(entries: Iterable[Dict[str, Any]]) -> Dict[str, List[Dict[str, Any]]]:
    grouped: Dict[str, List[Dict[str, Any]]] = {}
    for entry in entries:
        grouped.setdefault(str(entry["subsystem"]), []).append(entry)
    return dict(sorted(grouped.items()))


def _parse_float_list(value: str) -> List[float]:
    try:
        return [float(part) for part in re.split(r"[\s,]+", value.strip()) if part]
    except ValueError:
        return []


def identify_bottleneck_candidates(entries: Iterable[Dict[str, Any]]) -> List[Dict[str, str]]:
    """Heuristic notes for values that deserve manual review before tuning."""
    candidates: List[Dict[str, str]] = []
    for entry in entries:
        attrs = entry.get("attributes", {})
        name = str(entry.get("name", ""))
        subsystem = str(entry.get("subsystem", ""))
        kind = str(entry.get("kind", ""))

        force_range = attrs.get("actuatorfrcrange") or attrs.get("forcerange") or attrs.get("ctrlrange")
        if force_range:
            vals = _parse_float_list(force_range)
            if len(vals) >= 2:
                span = abs(vals[1] - vals[0])
                max_abs = max(abs(vals[0]), abs(vals[1]))
                if subsystem == "base_casters_wheels" and max_abs <= 25.0:
                    candidates.append(
                        {
                            "severity": "review",
                            "name": name,
                            "kind": kind,
                            "attribute": "force/control range",
                            "reason": f"base wheel/caster range {force_range!r} may clip visible whole-body motion",
                        }
                    )
                if "wrist" in name.lower() and max_abs <= 15.0:
                    candidates.append(
                        {
                            "severity": "review",
                            "name": name,
                            "kind": kind,
                            "attribute": "force/control range",
                            "reason": f"wrist range {force_range!r} is much smaller than shoulder/elbow ranges",
                        }
                    )
                if span == 0.0:
                    candidates.append(
                        {
                            "severity": "warning",
                            "name": name,
                            "kind": kind,
                            "attribute": "range",
                            "reason": "zero-width range disables useful motion/actuation",
                        }
                    )

        if "damping" in attrs:
            vals = _parse_float_list(attrs["damping"])
            if vals and vals[0] >= 1000.0:
                candidates.append(
                    {
                        "severity": "review",
                        "name": name,
                        "kind": kind,
                        "attribute": "damping",
                        "reason": f"very high damping={attrs['damping']!r}; likely hold-like behavior and not a compliance tuning target",
                    }
                )
            elif vals and subsystem == "base_casters_wheels" and vals[0] >= 1.0:
                candidates.append(
                    {
                        "severity": "note",
                        "name": name,
                        "kind": kind,
                        "attribute": "damping",
                        "reason": f"base damping={attrs['damping']!r} should be considered when diagnosing small base response",
                    }
                )

        if kind == "geom" and subsystem == "scene_contact":
            candidates.append(
                {
                    "severity": "note",
                    "name": name,
                    "kind": kind,
                    "attribute": "contact",
                    "reason": "floor/contact parameters can dominate mobile-base response; compare with MuJoCo defaults if friction is omitted",
                }
            )
    return candidates


def audit_paths(model: Path, scene: Optional[Path] = None) -> Dict[str, Any]:
    paths = [model]
    if scene is not None and scene != model:
        paths.append(scene)

    by_type: Dict[str, List[Dict[str, Any]]] = {
        "joints": [],
        "actuators": [],
        "geoms": [],
        "other_limit_fields": [],
    }
    for path in paths:
        parsed = parse_mjcf(path)
        for key, values in parsed.items():
            by_type[key].extend(values)

    all_entries = [entry for values in by_type.values() for entry in values]
    return {
        "model": str(model),
        "scene": str(scene) if scene is not None else None,
        "counts": {key: len(values) for key, values in by_type.items()},
        "by_type": by_type,
        "by_subsystem": group_entries(all_entries),
        "bottleneck_candidates": identify_bottleneck_candidates(all_entries),
    }


def write_markdown_report(audit: Dict[str, Any], path: Path) -> None:
    lines = [
        "# PR2 MJCF Force/Actuator Limit Audit",
        "",
        "This report is generated from the read-only MJCF audit helper. It identifies values that may limit or mask whole-body admittance motion; it does not recommend changing XML before simulation evidence is collected.",
        "",
        "## Input files",
        f"- Model: `{audit['model']}`",
        f"- Scene: `{audit.get('scene') or 'not provided'}`",
        "",
        "## Extracted entry counts",
        "",
    ]
    for key, count in audit["counts"].items():
        lines.append(f"- `{key}`: {count}")

    lines.extend(["", "## Bottleneck candidates", ""])
    candidates = audit.get("bottleneck_candidates", [])
    if not candidates:
        lines.append("No heuristic bottleneck candidates found. Manual review is still required.")
    else:
        for item in candidates:
            lines.append(
                f"- **{item['severity']}** `{item['name']}` ({item['kind']}, {item['attribute']}): {item['reason']}"
            )

    lines.extend(["", "## Subsystem summary", ""])
    for subsystem, entries in audit["by_subsystem"].items():
        lines.append(f"### {subsystem}")
        lines.append("")
        for entry in entries[:40]:
            attrs = ", ".join(f"{k}={v!r}" for k, v in entry["attributes"].items())
            lines.append(f"- `{entry['name']}` ({entry['kind']}/{entry['tag']}): {attrs}")
        if len(entries) > 40:
            lines.append(f"- ... {len(entries) - 40} more entries omitted from markdown summary; see JSON for full details.")
        lines.append("")

    lines.extend([
        "## Risk notes and next experiments",
        "",
        "- Keep acceptance and presentation/demo tuning separate; do not raise XML limits just to make a demo look larger without validating acceptance behavior.",
        "- Base caster/wheel force and velocity ranges should be tested with CSV metrics before changing admittance gains.",
        "- Very high torso damping is likely intentional hold behavior; changing it can destabilize posture and should be treated as a separate experiment.",
        "- Compare actuator `ctrlrange` with joint `actuatorfrcrange` before assuming which MuJoCo limit is binding.",
        "- Contact/friction parameters can dominate mobile-base translation; if floor friction is omitted, document the MuJoCo defaults used by the installed version.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path, help="PR2 robot MJCF/XML file, usually robot_pr2.xml")
    parser.add_argument("--scene", type=Path, help="Optional scene XML for floor/contact fields")
    parser.add_argument("--out", type=Path, help="Write full JSON audit to this path")
    parser.add_argument("--markdown", type=Path, help="Write a markdown summary report to this path")
    args = parser.parse_args()

    audit = audit_paths(args.model, args.scene)
    text = json.dumps(audit, indent=2, sort_keys=True)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    if args.markdown:
        write_markdown_report(audit, args.markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
