# Metric Definitions

## F1 Score

### Formula

```
Precision = TP / (TP + FP)
Recall    = TP / (TP + FN)
F1        = 2 * Precision * Recall / (Precision + Recall)
```

Where TP, FP, FN are determined by matching predictions to ground truth at a given confidence threshold and spatial criterion (see table below). We report the maximum F1 achieved across all confidence thresholds.

| Step | Standard 2D F1 | Our F1 |
|---|---|---|
| Confidence filter | Discard predictions below a fixed confidence threshold | Same |
| Spatial matching criterion | Bounding box IoU ≥ 0.5 | 3D Euclidean centroid distance ≤ 0.10 m |
| Match priority | Closest prediction wins (highest IoU) | Closest prediction wins (smallest distance) |
| Class constraint | Same class only | Same class only |
| What a "prediction" is | 2D bounding box from a single image | 3D voxel cluster accumulated across all viewpoints |
| F1 formula | Harmonic mean of precision and recall | Same |

## mAP

### Formula

```
AP(c, t)  = area under precision-recall curve for class c at threshold t
mAP       = mean over all classes c and all thresholds t of AP(c, t)
```

The PR curve for each (class, threshold) pair is built by sorting predictions by confidence descending and computing cumulative precision and recall after each prediction is added. A prediction is a TP if it is within threshold `t` of an unmatched same-class GT point; otherwise it is a FP.

- Standard COCO: thresholds `t` are IoU values {0.50, 0.55, ..., 0.95} (10 values)
- Our implementation: thresholds `t` are distances {0.03, 0.06, ..., 0.30} m (10 values)

| Step | Standard COCO Detection | Our Implementation |
|---|---|---|
| Spatial matching criterion | Bounding box IoU | 3D Euclidean centroid distance |
| Confidence handling | Sort predictions by confidence descending, build cumulative PR curve | Same |
| Match assignment | Greedy one-to-one per class, sorted by confidence | Same |
| Class constraint | Same class only | Same class only |
| Spatial threshold sweep | IoU thresholds from 0.50 to 0.95 in steps of 0.05 (10 thresholds) | Distance thresholds from 0.03 m to 0.30 m in steps of 0.03 m (10 thresholds) |
| Per-class AP | Area under PR curve at each IoU threshold | Area under PR curve at each distance threshold |
| Averaging | Mean over classes × IoU thresholds | Mean over classes × distance thresholds |
| Healthy trees (no GT, has predictions) | All predictions are FP → AP = 0 | Same |
