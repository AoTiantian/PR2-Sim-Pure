"""Validate lossless CSV evidence and recompute the two primary scalar metrics."""
import gzip
import hashlib
import io
import json
from pathlib import Path

import numpy as np


def main():
    parent = Path(__file__).resolve().parent.parent
    count = 0
    for directory in ["base_feedforward_20261027", "base_feedforward_paths_20261027"]:
        root = parent / directory
        manifest = json.loads((root / "evidence_manifest.json").read_text())
        summary = json.loads((root / "runtime_summary.json").read_text())
        assert set(manifest) == set(summary), "evidence and summary run lists differ"
        for tag, record in manifest.items():
            raw = gzip.decompress((root / record["archive"]).read_bytes())
            assert hashlib.sha256(raw).hexdigest() == record["original_csv_sha256"], tag
            a = np.genfromtxt(io.BytesIO(raw), delimiter=",", names=True, dtype=None, encoding="utf8")
            error = np.column_stack([a["position_error_"+axis] for axis in "xy"])
            force = np.column_stack([a["human_force_"+axis] for axis in "xy"])
            values = {"hand_xy_rmse_cm":float(100*np.sqrt(np.mean(np.sum(error**2,axis=1)))),
                      "human_xy_force_rms_N":float(np.sqrt(np.mean(np.sum(force**2,axis=1))))}
            for metric, value in values.items():
                assert np.isclose(value, summary[tag][metric], rtol=1e-12, atol=1e-12), (tag,metric)
            count += 1
    print(f"Verified {count} archived histories and their horizontal RMSE/force metrics.")


if __name__ == "__main__":
    main()
