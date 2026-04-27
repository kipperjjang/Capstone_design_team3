# Target Prediction Skeleton

This folder is an offline PyTorch scaffold for future target prediction.

The recommended structure is:

```text
Filtered Kalman state history -> AI model -> future target image position
```

The model does not replace the Kalman filter as a state estimator. The KF still cleans sparse vision data. The AI model directly predicts the future 2D image position from recent filtered KF state history.

## CSV Format

Each row should represent one timestamp from one experiment run.

Required columns:

```text
run_id,time,est_x,est_y,est_vx,est_vy,est_ax,est_ay,truth_x,truth_y
```

Optional columns:

```text
scenario_id,frequency_hz,post_hit
```

Column meaning:

```text
run_id       experiment/run identifier. Different runs must have different ids.
time         seconds from start of the run.
est_x/y      current input image position estimate [px].
est_vx/y     current input image velocity estimate [px/s].
est_ax/y     current input image acceleration estimate [px/s^2].
truth_x/y    ground-truth image position [px], used only for training labels.
```

Use a run-level split for validation/test. Do not randomly split neighboring frames from the same run into both train and validation, because that makes the result look better than it is.

## Generating Model-Only Training Data

Use this path when you want to train the predictor independently from the Kalman
filter. It writes the same CSV schema, but `est_*` comes from the generated
synthetic state instead of `/test/debug`.

The default mode is intentionally easy: deterministic trajectory, exact
position/velocity/acceleration input, no measurement noise, and no hidden
post-hit trajectory switch. Use it only as a sanity check that the model can
learn the clean motion family before adding estimator noise and real timing
effects.

Clean deterministic input:

```bash
cd ~/ros2_ws/src/capstone/src/scripts/prediction
python3 generate_synthetic_data.py \
  --output-csv data/synthetic_clean.csv \
  --scenario-count 500 \
  --periods-per-scenario 5.0 \
  --sample-rate 30
```

Harder noisy input without the Kalman node:

```bash
python3 generate_synthetic_data.py \
  --output-csv data/synthetic_noisy.csv \
  --scenario-count 500 \
  --state-source noisy_measurement \
  --position-noise-std 10 \
  --post-hit-switch
```

## Sampling Kalman-Filtered Training Data

Use this path only when you intentionally want the training CSV to contain Kalman
filter output. Run the normal estimator test node and this sampler together. The
sampler publishes synthetic vision trajectories and records only filtered Kalman
update samples where `predicted_only == false`.

Example:

```bash
ros2 run capstone test_node --ros-args -p config_path:=~/ros2_ws/src/capstone/src/config/params.yaml
python3 ~/ros2_ws/src/capstone/src/scripts/prediction/sample_training_data.py --ros-args \
  -p output_csv:=~/ros2_ws/src/capstone/src/scripts/prediction/data/training_samples.csv \
  -p scenario_count:=200 \
  -p periods_per_scenario:=3.0
```

## Training

From the ROS workspace source directory:

GRU baseline:

```bash
cd ~/ros2_ws/src/capstone/src/scripts/prediction
python3 gru/train.py \
  --csv data/*.csv \
  --out checkpoints/gru_target_predictor.pt \
  --horizon 1.0 \
  --window 1.0 \
  --sample-rate 30 \
  --hidden-dim 128 \
  --layers 2 \
  --dropout 0.1 \
  --epochs 1000
```

TCN model:

```bash
python3 tcn/train.py \
  --csv data/*.csv \
  --out checkpoints/tcn_target_predictor.pt \
  --horizon-secs 0.1 0.2 0.5 1.0 \
  --target-mode delta \
  --window 1.0 \
  --sample-rate 30 \
  --channels 128 128 128 128 \
  --kernel-size 5 \
  --dropout 0.1 \
  --epochs 1000
```

The old root-level `train.py`, `evaluate.py`, `model.py`, and `predictor.py`
remain as thin GRU compatibility wrappers. New experiments should prefer the
explicit `gru/` or `tcn/` path.

Important arguments:

```text
--horizon      future prediction horizon in seconds, e.g. 0.5 or 1.0.
--horizon-secs multiple future horizons for TCN training, e.g. 0.1 0.2 0.5 1.0.
--target-mode  TCN target type. The default is delta, which predicts future minus current position.
--window       past history duration used as model input.
--sample-rate  resampled history rate. 50-100 Hz is usually enough.
--channels     TCN output channels for each dilated residual level.
--kernel-size  TCN temporal convolution width.
```

The training target is:

```text
GRU absolute mode:
target = true_position(t + horizon)

TCN delta mode:
target_delta = true_position(t + horizon) - estimated_position(t)
```

At runtime:

```text
GRU absolute mode:
final_position = model(history, horizon)

TCN delta mode:
final_position = estimated_position(t) + model(history, horizon)
```

## Evaluation

Evaluate a trained checkpoint on held-out scenario CSV files:

GRU:

```bash
cd ~/ros2_ws/src/capstone/src/scripts/prediction
python3 gru/evaluate.py \
  --checkpoint checkpoints/gru_target_predictor.pt \
  --csv data/test/*.csv \
  --out-csv results/gru_test_predictions.csv \
  --plot results/gru_test_errors.png \
  --plot-2d results/gru_test_trajectory_2d.png
```

TCN:

```bash
python3 tcn/evaluate.py \
  --checkpoint checkpoints/tcn_target_predictor.pt \
  --csv data/test/*.csv \
  --out-csv results/tcn_test_predictions.csv \
  --plot results/tcn_test_errors.png \
  --plot-2d results/tcn_test_trajectory_2d.png
```

The evaluator reports:

```text
GRU future-position RMSE
TCN delta-prediction RMSE/MAE
TCN reconstructed absolute-position RMSE/MAE
constant-acceleration KF baseline RMSE
RMSE improvement versus the baseline
```

For an intuitive single-scenario view, use `--plot-2d`. It draws the truth
trajectory in image coordinates and overlays sampled current input points, true
future positions, model predictions, and error segments. Use `--plot-2d-run
scenario_0003` to choose a specific run.

Use CSV files from runs/scenarios that were not used for training.

## Integration Idea

Later, make a ROS2 prediction node that:

```text
1. Subscribes to KF/debug state.
2. Stores the recent history window.
3. Computes the hit horizon: actuator delay + projectile flight time + control delay.
4. Calls TargetPredictor.
5. Publishes the predicted image point for aiming.
```
