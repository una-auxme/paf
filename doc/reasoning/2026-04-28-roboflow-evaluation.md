# Roboflow Evaluation Notes

## Task

Evaluate whether the public Roboflow organization repositories are useful for PAF's current perception work and create concrete follow-up issues only where the fit is strong enough.

## Sources inspected

- `code/perception/perception/vision_node.py`
- `code/perception/traffic_light_detection/dataset.dvc`
- `code/perception/traffic_light_detection/dvc.yaml`
- `doc/perception/vision_node.md`
- `doc/perception/traffic_light_detection.md`
- `doc/perception/experiments/object-detection-model_evaluation/README.md`
- `https://github.com/roboflow`
- `https://github.com/roboflow/supervision`
- `https://github.com/roboflow/notebooks`
- `https://github.com/roboflow/inference`
- `https://github.com/una-auxme/paf/issues/918`

## Main conclusion

Roboflow is only partially helpful for PAF right now.

The useful part is not adopting another runtime inference stack. The useful part is borrowing tooling and workflows that reduce friction around perception evaluation and dataset iteration.

The current PAF perception stack already has a direct ROS2 + Ultralytics path:

- the vision node runs Ultralytics YOLO segmentation models,
- traffic-light handling already has a dedicated local training pipeline,
- traffic-light data is already stored in a local DVC-managed dataset.

That means the main bottleneck is dataset quality, evaluation speed, and debugging support, not model serving.

## What looks useful

1. `roboflow/supervision` looks like the best fit.
   It is model-agnostic, works with common detection outputs, and would be most useful as an offline helper library for overlays, dataset utilities, simple tracking experiments, and perception debugging around the existing Ultralytics outputs.
2. Selected `roboflow/notebooks` are useful as experiment references.
   The strongest candidates are the notebooks around auto-annotation, Grounding DINO plus SAM style dataset bootstrapping, and YOLO fine-tuning workflows.

## What does not look useful yet

`roboflow/inference` is not a good near-term integration target.

Reasons:

- PAF already has a ROS-native runtime stack and direct model loading.
- Adding another inference server and workflow layer increases system complexity.
- Cloud-connected features add account, licensing, and deployment concerns that do not address the current bottleneck.
- The present need is better local evaluation and better datasets, not a new serving boundary.

## Recommended follow-up work

1. Evaluate `supervision` as an experiments-only dependency for offline perception benchmarking, visual debugging, and tracker-style analysis on recorded or generated data.
   This is now tracked in `una-auxme/paf#919`.
2. Evaluate a semi-automatic labeling workflow inspired by Roboflow notebooks for expanding PAF datasets while keeping data local and DVC-managed.
   This is now tracked in `una-auxme/paf#920`.
3. Defer any `roboflow/inference` adoption unless PAF later develops a clear need for standalone CV microservices or remote model-serving workflows.
