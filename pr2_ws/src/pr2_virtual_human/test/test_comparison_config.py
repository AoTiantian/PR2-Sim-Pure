from pathlib import Path
import yaml

from pr2_virtual_human.comparison_config import load_comparison_config


def test_default_config_is_valid_and_hash_is_stable() -> None:
    path = Path(__file__).parents[1] / "config" / "transport_comparison.yaml"
    first = load_comparison_config(path)
    second = load_comparison_config(path)
    assert first.config_hash == second.config_hash
    assert first.trajectory.duration_sec == first.timing.tracking_sec
    assert first.hand_offset.tolist() == [1.0, 0.0, 0.0]


def test_angular_frame_is_explicit_and_legacy_configs_remain_world_frame(tmp_path) -> None:
    source = Path(__file__).parents[1] / "config" / "transport_comparison.yaml"
    raw = yaml.safe_load(source.read_text())
    path = tmp_path / "config.yaml"
    raw["human_impedance"]["angular_frame"] = "body"
    path.write_text(yaml.safe_dump(raw))
    assert load_comparison_config(path).human_impedance.angular_frame == "body"
    del raw["human_impedance"]["angular_frame"]
    path.write_text(yaml.safe_dump(raw))
    assert load_comparison_config(path).human_impedance.angular_frame == "world"

