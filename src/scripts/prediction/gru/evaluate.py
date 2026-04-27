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
  from .model import FutureGRUPredictor
except ImportError:
  sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
  from dataset import PredictionWindowDataset, Standardizer
  from gru.model import FutureGRUPredictor


def parse_args():
  parser = argparse.ArgumentParser(description='Evaluate trained GRU future target predictor.')
  parser.add_argument('--checkpoint', required=True, help='Checkpoint saved by train.py.')
  parser.add_argument('--csv', nargs='+', required=True, help='Held-out CSV log files.')
  parser.add_argument('--batch-size', type=int, default=256)
  parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
  parser.add_argument('--out-csv', default='', help='Optional per-window prediction CSV.')
  parser.add_argument('--plot', default='', help='Optional output PNG path for summary plots.')
  parser.add_argument('--plot-2d', default='', help='Optional output PNG path for one 2D trajectory plot.')
  parser.add_argument('--plot-2d-run', default='', help='Run/scenario id to draw. Defaults to the first evaluated run.')
  parser.add_argument('--plot-2d-samples', type=int, default=40, help='Number of prediction points to overlay.')
  parser.add_argument('--show', action='store_true', help='Show matplotlib plot interactively.')
  return parser.parse_args()


def load_model(checkpoint_path, device):
  checkpoint = torch.load(checkpoint_path, map_location=device)
  model = FutureGRUPredictor(**checkpoint['model_config']).to(device)
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
      'mean': float('nan'),
      'median': float('nan'),
      'p90': float('nan'),
      'max': float('nan'),
    }
  return {
    'name': name,
    'count': int(errors.size),
    'rmse': float(np.sqrt(np.mean(errors * errors))),
    'mean': float(np.mean(errors)),
    'median': float(np.median(errors)),
    'p90': float(np.percentile(errors, 90.0)),
    'max': float(np.max(errors)),
  }


def print_summary(summary):
  print(
    '%s: count=%d rmse=%.3fpx mean=%.3fpx median=%.3fpx p90=%.3fpx max=%.3fpx' %
    (
      summary['name'],
      summary['count'],
      summary['rmse'],
      summary['mean'],
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
  ax_hist.hist(model_errors, bins=40, alpha=0.65, color='tab:blue', label='GRU model')
  ax_hist.set_xlabel('future position error [px]')
  ax_hist.set_ylabel('count')
  ax_hist.set_title('Error distribution')
  ax_hist.grid(True, alpha=0.3)
  ax_hist.legend()

  times = np.asarray([row['anchor_time'] for row in rows], dtype=float)
  ax_time.scatter(times, ca_errors, s=8, alpha=0.35, color='tab:red', label='CA baseline')
  ax_time.scatter(times, model_errors, s=8, alpha=0.45, color='tab:blue', label='GRU model')
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

  ax.set_title(
    '%s  horizon=%.3fs  mean error=%.2fpx' %
    (selected_run_id, sampled_rows[0].get('horizon_sec', 0.0), float(np.mean(errors))))
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

  feature_standardizer = Standardizer.from_state_dict(checkpoint['feature_standardizer'])
  target_standardizer = Standardizer.from_state_dict(checkpoint['target_standardizer'])
  dataset = PredictionWindowDataset(
    args.csv,
    horizon_sec=float(checkpoint['horizon_sec']),
    window_sec=float(checkpoint['window_sec']),
    sample_rate_hz=float(checkpoint['sample_rate_hz']),
    feature_standardizer=feature_standardizer,
    target_standardizer=target_standardizer)
  loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)

  rows = []
  model_errors = []
  ca_errors = []

  with torch.no_grad():
    for batch in loader:
      history = batch['history'].to(args.device)
      horizon = batch['horizon'].to(args.device)
      pred_norm = model(history, horizon)
      pred_xy = target_standardizer.inverse_transform(pred_norm.cpu().numpy())
      truth_xy = batch['truth_future'].cpu().numpy()
      ca_xy = batch['ca_future'].cpu().numpy()
      sample_indices = batch['sample_index'].cpu().numpy()

      batch_model_errors = np.linalg.norm(pred_xy - truth_xy, axis=1)
      batch_ca_errors = np.linalg.norm(ca_xy - truth_xy, axis=1)
      model_errors.extend(batch_model_errors.tolist())
      ca_errors.extend(batch_ca_errors.tolist())

      for local_idx, sample_idx in enumerate(sample_indices):
        sample = dataset.samples[int(sample_idx)]
        est_x_idx = dataset.feature_columns.index('est_x')
        est_y_idx = dataset.feature_columns.index('est_y')
        current_xy = sample['history'][-1, [est_x_idx, est_y_idx]]
        rows.append({
          'run_id': sample['run_id'],
          'anchor_time': sample['anchor_time'],
          'horizon_sec': float(checkpoint['horizon_sec']),
          'current_x': float(current_xy[0]),
          'current_y': float(current_xy[1]),
          'truth_x': float(truth_xy[local_idx, 0]),
          'truth_y': float(truth_xy[local_idx, 1]),
          'pred_x': float(pred_xy[local_idx, 0]),
          'pred_y': float(pred_xy[local_idx, 1]),
          'ca_x': float(ca_xy[local_idx, 0]),
          'ca_y': float(ca_xy[local_idx, 1]),
          'model_error_px': float(batch_model_errors[local_idx]),
          'ca_error_px': float(batch_ca_errors[local_idx]),
        })

  model_summary = summarize_errors('GRU model', model_errors)
  ca_summary = summarize_errors('CA baseline', ca_errors)
  print('checkpoint:', args.checkpoint)
  print('horizon_sec:', checkpoint['horizon_sec'])
  print('window_sec:', checkpoint['window_sec'])
  print('sample_rate_hz:', checkpoint['sample_rate_hz'])
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
