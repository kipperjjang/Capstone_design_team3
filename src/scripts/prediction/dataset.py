#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset


FEATURE_COLUMNS = [
  'age_sec',
  'est_x',
  'est_y',
  'est_vx',
  'est_vy',
  'est_ax',
  'est_ay',
]


REQUIRED_COLUMNS = [
  'run_id',
  'time',
  'est_x',
  'est_y',
  'est_vx',
  'est_vy',
  'est_ax',
  'est_ay',
  'truth_x',
  'truth_y',
]


class Standardizer:
  def __init__(self, mean=None, std=None):
    self.mean = mean
    self.std = std

  def fit(self, values):
    arr = np.asarray(values, dtype=np.float32)
    self.mean = arr.mean(axis=0)
    self.std = arr.std(axis=0)
    self.std[self.std < 1e-6] = 1.0
    return self

  def transform(self, values):
    return (np.asarray(values, dtype=np.float32) - self.mean) / self.std

  def inverse_transform(self, values):
    return np.asarray(values, dtype=np.float32) * self.std + self.mean

  def state_dict(self):
    return {
      'mean': self.mean.tolist(),
      'std': self.std.tolist(),
    }

  @classmethod
  def from_state_dict(cls, state):
    return cls(
      mean=np.asarray(state['mean'], dtype=np.float32),
      std=np.asarray(state['std'], dtype=np.float32))


def _to_float(row, key, default=0.0):
  value = row.get(key, '')
  if value is None or value == '':
    return default
  return float(value)


def _read_csv_files(paths):
  rows_by_run = {}
  for path in paths:
    path = Path(path)
    with path.open('r', newline='') as f:
      reader = csv.DictReader(f)
      missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
      if missing:
        raise ValueError('%s is missing required columns: %s' % (path, ', '.join(missing)))

      for row in reader:
        run_id = row.get('run_id') or path.stem
        rows_by_run.setdefault(run_id, []).append(row)
  return rows_by_run


def _prepare_run(rows):
  rows = sorted(rows, key=lambda row: _to_float(row, 'time'))
  time = np.asarray([_to_float(row, 'time') for row in rows], dtype=np.float32)

  values = {
    'time': time,
    'est_x': np.asarray([_to_float(row, 'est_x') for row in rows], dtype=np.float32),
    'est_y': np.asarray([_to_float(row, 'est_y') for row in rows], dtype=np.float32),
    'est_vx': np.asarray([_to_float(row, 'est_vx') for row in rows], dtype=np.float32),
    'est_vy': np.asarray([_to_float(row, 'est_vy') for row in rows], dtype=np.float32),
    'est_ax': np.asarray([_to_float(row, 'est_ax') for row in rows], dtype=np.float32),
    'est_ay': np.asarray([_to_float(row, 'est_ay') for row in rows], dtype=np.float32),
    'truth_x': np.asarray([_to_float(row, 'truth_x') for row in rows], dtype=np.float32),
    'truth_y': np.asarray([_to_float(row, 'truth_y') for row in rows], dtype=np.float32),
  }
  return values


def _interp(run, column, query_times):
  return np.interp(query_times, run['time'], run[column]).astype(np.float32)


def _normalize_horizons(horizon_sec=None, horizon_secs=None):
  if horizon_secs is None:
    horizon_secs = [0.5 if horizon_sec is None else horizon_sec]
  values = [float(value) for value in horizon_secs]
  if not values:
    raise ValueError('At least one prediction horizon is required.')
  if any(value <= 0.0 for value in values):
    raise ValueError('Prediction horizons must be positive.')
  return values


class PredictionWindowDataset(Dataset):
  def __init__(
      self,
      csv_paths,
      horizon_sec=0.5,
      horizon_secs=None,
      window_sec=0.8,
      sample_rate_hz=50.0,
      run_ids=None,
      feature_standardizer=None,
      target_standardizer=None,
      target_mode='absolute'):
    if target_mode not in ('absolute', 'delta'):
      raise ValueError("target_mode must be 'absolute' or 'delta'.")

    self.horizon_secs = _normalize_horizons(horizon_sec, horizon_secs)
    self.horizon_sec = self.horizon_secs[0]
    self.max_horizon_sec = max(self.horizon_secs)
    self.target_mode = target_mode
    self.window_sec = float(window_sec)
    self.sample_rate_hz = float(sample_rate_hz)
    self.feature_columns = list(FEATURE_COLUMNS)
    self.sequence_len = int(math.floor(self.window_sec * self.sample_rate_hz)) + 1

    rows_by_run = _read_csv_files(csv_paths)
    selected_run_ids = set(run_ids) if run_ids is not None else set(rows_by_run.keys())

    self.samples = []
    feature_values_for_fit = []
    target_values_for_fit = []

    for run_id, rows in rows_by_run.items():
      if run_id not in selected_run_ids:
        continue
      run = _prepare_run(rows)
      if len(run['time']) < 2:
        continue
      first_anchor = run['time'][0] + self.window_sec
      last_anchor = run['time'][-1]
      anchor_times = run['time'][(run['time'] >= first_anchor) & (run['time'] <= last_anchor)]

      for anchor_time in anchor_times:
        if anchor_time - self.window_sec < run['time'][0] - 1e-6:
          continue

        history_times = np.linspace(anchor_time - self.window_sec, anchor_time, self.sequence_len)
        history_columns = []
        for column in self.feature_columns:
          if column == 'age_sec':
            history_columns.append((history_times - anchor_time).astype(np.float32))
          else:
            history_columns.append(_interp(run, column, history_times))
        history = np.stack(history_columns, axis=-1)

        current_position = np.asarray([
          _interp(run, 'est_x', [anchor_time])[0],
          _interp(run, 'est_y', [anchor_time])[0],
        ], dtype=np.float32)
        current_velocity = np.asarray([
          _interp(run, 'est_vx', [anchor_time])[0],
          _interp(run, 'est_vy', [anchor_time])[0],
        ], dtype=np.float32)
        current_acceleration = np.asarray([
          _interp(run, 'est_ax', [anchor_time])[0],
          _interp(run, 'est_ay', [anchor_time])[0],
        ], dtype=np.float32)

        # Horizon expansion: each anchor can contribute one sample per valid
        # future horizon while reusing the same past trajectory window.
        for horizon in self.horizon_secs:
          future_time = float(anchor_time) + float(horizon)
          if future_time > float(run['time'][-1]) + 1e-6:
            continue
          future_time = min(future_time, float(run['time'][-1]))

          truth_future = np.asarray([
            _interp(run, 'truth_x', [future_time])[0],
            _interp(run, 'truth_y', [future_time])[0],
          ], dtype=np.float32)
          ca_future = (
            current_position
            + current_velocity * horizon
            + 0.5 * current_acceleration * horizon * horizon)
          target_delta = truth_future - current_position
          # Target construction: delta mode learns displacement from the
          # current estimate; absolute mode preserves the previous behavior.
          target = target_delta if self.target_mode == 'delta' else truth_future

          self.samples.append({
            'history': history,
            'target': target.astype(np.float32),
            'target_delta': target_delta.astype(np.float32),
            'truth_future': truth_future,
            'current_position': current_position,
            'ca_future': ca_future.astype(np.float32),
            'run_id': run_id,
            'anchor_time': float(anchor_time),
            'horizon_sec': float(horizon),
          })
          feature_values_for_fit.append(history.reshape(-1, history.shape[-1]))
          target_values_for_fit.append(target)

    if not self.samples:
      raise ValueError('No prediction windows were created. Check time range, window, and horizon.')

    if feature_standardizer is None:
      self.feature_standardizer = Standardizer().fit(np.concatenate(feature_values_for_fit, axis=0))
    else:
      self.feature_standardizer = feature_standardizer

    if target_standardizer is None:
      self.target_standardizer = Standardizer().fit(np.stack(target_values_for_fit, axis=0))
    else:
      self.target_standardizer = target_standardizer

  def __len__(self):
    return len(self.samples)

  def __getitem__(self, idx):
    sample = self.samples[idx]
    history = self.feature_standardizer.transform(sample['history'])
    target = self.target_standardizer.transform(sample['target'])
    return {
      'history': torch.tensor(history, dtype=torch.float32),
      'horizon': torch.tensor(sample['horizon_sec'], dtype=torch.float32),
      'target': torch.tensor(target, dtype=torch.float32),
      'target_delta': torch.tensor(sample['target_delta'], dtype=torch.float32),
      'truth_future': torch.tensor(sample['truth_future'], dtype=torch.float32),
      'current_position': torch.tensor(sample['current_position'], dtype=torch.float32),
      'ca_future': torch.tensor(sample['ca_future'], dtype=torch.float32),
      'sample_index': torch.tensor(idx, dtype=torch.long),
    }


def split_run_ids(csv_paths, val_fraction=0.2, seed=0):
  rows_by_run = _read_csv_files(csv_paths)
  run_ids = sorted(rows_by_run.keys())
  rng = np.random.default_rng(seed)
  rng.shuffle(run_ids)
  val_count = max(1, int(round(len(run_ids) * val_fraction))) if len(run_ids) > 1 else 0
  val_ids = set(run_ids[:val_count])
  train_ids = set(run_ids[val_count:])
  if not train_ids:
    train_ids = set(run_ids)
  return sorted(train_ids), sorted(val_ids)
