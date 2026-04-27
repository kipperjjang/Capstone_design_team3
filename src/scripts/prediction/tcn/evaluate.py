#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

try:
  from ..dataset import PredictionWindowDataset, Standardizer
  from .model import FutureTCNPredictor
except ImportError:
  sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
  from dataset import PredictionWindowDataset, Standardizer
  from tcn.model import FutureTCNPredictor


def parse_args():
  parser = argparse.ArgumentParser(description='Evaluate trained TCN future target predictor.')
  parser.add_argument('--checkpoint', required=True, help='Checkpoint saved by train.py.')
  parser.add_argument('--csv', nargs='+', required=True, help='Held-out CSV log files.')
  parser.add_argument('--horizon', type=float, default=None, help='Single prediction horizon [s]. Kept for compatibility.')
  parser.add_argument('--horizon-secs', type=float, nargs='+', default=None, help='Prediction horizons [s]. Defaults to checkpoint horizons.')
  parser.add_argument('--target-mode', choices=['delta', 'absolute'], default=None, help='Defaults to checkpoint target mode.')
  parser.add_argument('--batch-size', type=int, default=256)
  parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
  parser.add_argument('--out-csv', default='', help='Optional per-window prediction CSV.')
  parser.add_argument('--plot', default='', help='Optional output PNG path for summary plots.')
  parser.add_argument('--plot-2d', default='', help='Optional output PNG path for one 2D trajectory plot.')
  parser.add_argument('--plot-2d-run', default='', help='Run/scenario id to draw. Defaults to the first evaluated run.')
  parser.add_argument('--plot-2d-samples', type=int, default=40, help='Number of prediction points to overlay.')
  parser.add_argument('--show', action='store_true', help='Show matplotlib plot interactively.')
  return parser.parse_args()


def resolve_horizon_secs(args, checkpoint):
  if args.horizon_secs is not None:
    values = args.horizon_secs
  elif args.horizon is not None:
    values = [args.horizon]
  elif 'horizon_secs' in checkpoint:
    values = checkpoint['horizon_secs']
  else:
    values = [checkpoint.get('horizon_sec', 0.5)]
  values = [float(value) for value in values]
  if not values or any(value <= 0.0 for value in values):
    raise ValueError('Prediction horizons must be positive.')
  return values


def load_model(checkpoint_path, device):
  checkpoint = torch.load(checkpoint_path, map_location=device)
  model = FutureTCNPredictor(**checkpoint['model_config']).to(device)
  model.load_state_dict(checkpoint['model_state'])
  model.eval()
  return model, checkpoint


def summarize_errors(name, errors):
  errors = np.asarray(errors, dtype=np.float64)
  if errors.size == 0:
    return {
      'name': name,
      'count': 0,
      'rmse': float('nan'),
      'mae': float('nan'),
      'mean': float('nan'),
      'median': float('nan'),
      'p90': float('nan'),
      'max': float('nan'),
    }
  return {
    'name': name,
    'count': int(errors.size),
    'rmse': float(np.sqrt(np.mean(errors * errors))),
    'mae': float(np.mean(errors)),
    'mean': float(np.mean(errors)),
    'median': float(np.median(errors)),
    'p90': float(np.percentile(errors, 90.0)),
    'max': float(np.max(errors)),
  }


def print_summary(summary):
  print(
    '%s: count=%d rmse=%.3fpx mae=%.3fpx median=%.3fpx p90=%.3fpx max=%.3fpx' %
    (
      summary['name'],
      summary['count'],
      summary['rmse'],
      summary['mae'],
      summary['median'],
      summary['p90'],
      summary['max']))


def write_predictions(path, rows):
  path = Path(path).expanduser()
  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=[
      'run_id',
      'anchor_time',
      'horizon_sec',
      'current_x',
      'current_y',
      'truth_x',
      'truth_y',
      'pred_x',
      'pred_y',
      'ca_x',
      'ca_y',
      'target_dx',
      'target_dy',
      'pred_dx',
      'pred_dy',
      'delta_error_px',
      'model_error_px',
      'ca_error_px',
    ])
    writer.writeheader()
    for row in rows:
      writer.writerow(row)


def plot_results(path, rows, model_errors, ca_errors, show):
  import matplotlib.pyplot as plt

  fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
  ax_hist, ax_time = axes

  ax_hist.hist(ca_errors, bins=40, alpha=0.45, color='tab:red', label='CA baseline')
  ax_hist.hist(model_errors, bins=40, alpha=0.65, color='tab:blue', label='TCN model')
  ax_hist.set_xlabel('future position error [px]')
  ax_hist.set_ylabel('count')
  ax_hist.set_title('Error distribution')
  ax_hist.grid(True, alpha=0.3)
  ax_hist.legend()

  times = np.asarray([row['anchor_time'] for row in rows], dtype=float)
  ax_time.scatter(times, ca_errors, s=8, alpha=0.35, color='tab:red', label='CA baseline')
  ax_time.scatter(times, model_errors, s=8, alpha=0.45, color='tab:blue', label='TCN model')
  ax_time.set_xlabel('anchor time [s]')
  ax_time.set_ylabel('future position error [px]')
  ax_time.set_title('Error over scenario time')
  ax_time.grid(True, alpha=0.3)
  ax_time.legend()

  if path:
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    print('saved plot:', path)
  if show:
    plt.show()
  else:
    plt.close(fig)


def _row_float(row, key, default=0.0):
  value = row.get(key, '')
  if value is None or value == '':
    return default
  return float(value)


def read_truth_runs(csv_paths):
  runs = {}
  for csv_path in csv_paths:
    csv_path = Path(csv_path).expanduser()
    with csv_path.open('r', newline='') as f:
      reader = csv.DictReader(f)
      for row in reader:
        run_id = row.get('run_id') or csv_path.stem
        runs.setdefault(run_id, []).append({
          'time': _row_float(row, 'time'),
          'truth_x': _row_float(row, 'truth_x'),
          'truth_y': _row_float(row, 'truth_y'),
        })

  for run_rows in runs.values():
    run_rows.sort(key=lambda row: row['time'])
  return runs


def pick_evenly_spaced_rows(rows, max_count):
  if max_count <= 0 or len(rows) <= max_count:
    return list(rows)

  indices = np.linspace(0, len(rows) - 1, max_count)
  indices = np.unique(np.round(indices).astype(int))
  return [rows[int(idx)] for idx in indices]


def plot_trajectory_2d(path, rows, csv_paths, run_id, max_predictions, show):
  import matplotlib.pyplot as plt

  if not rows:
    print('skipped 2D plot: no evaluated prediction rows')
    return

  truth_runs = read_truth_runs(csv_paths)
  available_run_ids = [row['run_id'] for row in rows]
  selected_run_id = run_id or available_run_ids[0]
  if selected_run_id not in truth_runs:
    print('skipped 2D plot: run_id not found in CSV:', selected_run_id)
    return

  run_prediction_rows = [row for row in rows if row['run_id'] == selected_run_id]
  if not run_prediction_rows:
    print('skipped 2D plot: run_id has no prediction windows:', selected_run_id)
    return

  run_prediction_rows.sort(key=lambda row: row['anchor_time'])
  sampled_rows = pick_evenly_spaced_rows(run_prediction_rows, max_predictions)

  truth_rows = truth_runs[selected_run_id]
  truth_x = np.asarray([row['truth_x'] for row in truth_rows], dtype=float)
  truth_y = np.asarray([row['truth_y'] for row in truth_rows], dtype=float)

  current_x = np.asarray([row['current_x'] for row in sampled_rows], dtype=float)
  current_y = np.asarray([row['current_y'] for row in sampled_rows], dtype=float)
  future_x = np.asarray([row['truth_x'] for row in sampled_rows], dtype=float)
  future_y = np.asarray([row['truth_y'] for row in sampled_rows], dtype=float)
  pred_x = np.asarray([row['pred_x'] for row in sampled_rows], dtype=float)
  pred_y = np.asarray([row['pred_y'] for row in sampled_rows], dtype=float)
  errors = np.asarray([row['model_error_px'] for row in sampled_rows], dtype=float)

  fig, ax = plt.subplots(figsize=(8.0, 6.0), constrained_layout=True)
  ax.plot(truth_x, truth_y, color='0.15', linewidth=1.6, label='truth trajectory')
  ax.scatter(current_x, current_y, s=16, color='0.55', alpha=0.75, label='current input')
  ax.scatter(future_x, future_y, s=28, color='tab:green', alpha=0.9, label='true future')
  ax.scatter(pred_x, pred_y, s=36, marker='x', color='tab:blue', linewidths=1.6, label='model prediction')

  for row in sampled_rows:
    ax.plot(
      [row['truth_x'], row['pred_x']],
      [row['truth_y'], row['pred_y']],
      color='tab:blue',
      alpha=0.25,
      linewidth=0.8)

  horizons = sorted(set(round(float(row.get('horizon_sec', 0.0)), 3) for row in sampled_rows))
  horizon_label = 'multi-horizon' if len(horizons) > 1 else 'horizon=%.3fs' % horizons[0]
  ax.set_title(
    '%s  %s  mean error=%.2fpx' %
    (selected_run_id, horizon_label, float(np.mean(errors))))
  ax.set_xlabel('image x [px]')
  ax.set_ylabel('image y [px]')
  ax.grid(True, alpha=0.25)
  ax.axis('equal')
  ax.invert_yaxis()
  ax.legend(loc='best')

  if path:
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    print('saved 2D trajectory plot:', path)
    print('2D plot run_id:', selected_run_id)
  if show:
    plt.show()
  else:
    plt.close(fig)


def main():
  args = parse_args()
  model, checkpoint = load_model(args.checkpoint, args.device)
  horizon_secs = resolve_horizon_secs(args, checkpoint)
  target_mode = args.target_mode or checkpoint.get('target_mode', 'absolute')

  feature_standardizer = Standardizer.from_state_dict(checkpoint['feature_standardizer'])
  target_standardizer = Standardizer.from_state_dict(checkpoint['target_standardizer'])
  dataset = PredictionWindowDataset(
    args.csv,
    horizon_secs=horizon_secs,
    window_sec=float(checkpoint['window_sec']),
    sample_rate_hz=float(checkpoint['sample_rate_hz']),
    feature_standardizer=feature_standardizer,
    target_standardizer=target_standardizer,
    target_mode=target_mode)
  loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)

  rows = []
  delta_errors = []
  model_errors = []
  ca_errors = []

  with torch.no_grad():
    for batch in loader:
      history = batch['history'].to(args.device)
      horizon = batch['horizon'].to(args.device)
      pred_norm = model(history, horizon)
      pred_target = target_standardizer.inverse_transform(pred_norm.cpu().numpy())
      truth_xy = batch['truth_future'].cpu().numpy()
      current_xy = batch['current_position'].cpu().numpy()
      target_delta_xy = batch['target_delta'].cpu().numpy()
      if target_mode == 'delta':
        pred_delta_xy = pred_target
        pred_xy = current_xy + pred_delta_xy
      else:
        pred_xy = pred_target
        pred_delta_xy = pred_xy - current_xy
      ca_xy = batch['ca_future'].cpu().numpy()
      sample_indices = batch['sample_index'].cpu().numpy()

      batch_delta_errors = np.linalg.norm(pred_delta_xy - target_delta_xy, axis=1)
      batch_model_errors = np.linalg.norm(pred_xy - truth_xy, axis=1)
      batch_ca_errors = np.linalg.norm(ca_xy - truth_xy, axis=1)
      delta_errors.extend(batch_delta_errors.tolist())
      model_errors.extend(batch_model_errors.tolist())
      ca_errors.extend(batch_ca_errors.tolist())

      for local_idx, sample_idx in enumerate(sample_indices):
        sample = dataset.samples[int(sample_idx)]
        rows.append({
          'run_id': sample['run_id'],
          'anchor_time': sample['anchor_time'],
          'horizon_sec': sample['horizon_sec'],
          'current_x': float(current_xy[local_idx, 0]),
          'current_y': float(current_xy[local_idx, 1]),
          'truth_x': float(truth_xy[local_idx, 0]),
          'truth_y': float(truth_xy[local_idx, 1]),
          'pred_x': float(pred_xy[local_idx, 0]),
          'pred_y': float(pred_xy[local_idx, 1]),
          'ca_x': float(ca_xy[local_idx, 0]),
          'ca_y': float(ca_xy[local_idx, 1]),
          'target_dx': float(target_delta_xy[local_idx, 0]),
          'target_dy': float(target_delta_xy[local_idx, 1]),
          'pred_dx': float(pred_delta_xy[local_idx, 0]),
          'pred_dy': float(pred_delta_xy[local_idx, 1]),
          'delta_error_px': float(batch_delta_errors[local_idx]),
          'model_error_px': float(batch_model_errors[local_idx]),
          'ca_error_px': float(batch_ca_errors[local_idx]),
        })

  delta_summary = summarize_errors('TCN delta', delta_errors)
  model_summary = summarize_errors('TCN absolute', model_errors)
  ca_summary = summarize_errors('CA baseline', ca_errors)
  print('checkpoint:', args.checkpoint)
  print('horizon_secs:', horizon_secs)
  print('target_mode:', target_mode)
  print('window_sec:', checkpoint['window_sec'])
  print('sample_rate_hz:', checkpoint['sample_rate_hz'])
  print_summary(delta_summary)
  print_summary(model_summary)
  print_summary(ca_summary)
  if np.isfinite(ca_summary['rmse']) and ca_summary['rmse'] > 1e-9:
    improvement = 100.0 * (ca_summary['rmse'] - model_summary['rmse']) / ca_summary['rmse']
    print('rmse_improvement_vs_ca: %.2f%%' % improvement)

  if args.out_csv:
    write_predictions(args.out_csv, rows)
    print('saved predictions:', Path(args.out_csv).expanduser())
  if args.plot or args.show:
    plot_results(args.plot, rows, np.asarray(model_errors), np.asarray(ca_errors), args.show)
  if args.plot_2d:
    plot_trajectory_2d(
      args.plot_2d,
      rows,
      args.csv,
      args.plot_2d_run,
      args.plot_2d_samples,
      args.show)


if __name__ == '__main__':
  main()
