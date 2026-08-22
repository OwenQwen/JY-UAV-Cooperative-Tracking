# Model weights

YOLO weights are intentionally not committed to Git. Before publishing a
weight file, confirm that the training dataset, base model, and resulting
weights may legally be redistributed. Prefer a versioned GitHub Release asset;
use Git LFS only when the team accepts its storage and bandwidth limits.

The archived environment contained two distinct files:

| Archive name | SHA-256 |
| --- | --- |
| `best.pt` | `92BF01842048414B3555635E86A05A24979E5D816B46C093F899535BC1C0096F` |
| `best(1).pt` | `36B548A835067407E8B69DE7CD0B355DF210EE0ABB9BA4CA6276E471ACCA941B` |

Their provenance and final/obsolete status could not be determined from the
archive, so neither is silently labelled as the official model.

After obtaining an authorized model, configure the nodes explicitly:

```bash
export MODEL_PATH=/absolute/path/to/best.pt
```
