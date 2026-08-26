from pathlib import Path

from pr2_virtual_human.comparison_config import load_comparison_config


def test_default_config_is_valid_and_hash_is_stable() -> None:
    path = Path(__file__).parents[1] / "config" / "transport_comparison.yaml"
    first = load_comparison_config(path)
    second = load_comparison_config(path)
    assert first.config_hash == second.config_hash
    assert first.trajectory.duration_sec == first.timing.tracking_sec
    assert first.hand_offset.tolist() == [1.0, 0.0, 0.0]

