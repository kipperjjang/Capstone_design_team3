#!/usr/bin/env python3
import sys
from pathlib import Path

import numpy as np
import torch

try:
  from ..dataset import Standardizer
  from .model import FutureTCNPredictor
except ImportError:
  sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
  from dataset import Standardizer
  from tcn.model import FutureTCNPredictor


class TargetPredictor:
  def __init__(self, checkpoint_path, device='cpu'):
    self.device = device
    checkpoint = torch.load(checkpoint_path, map_location=device)
    self.feature_columns = checkpoint['feature_columns']
    self.feature_standardizer = Standardizer.from_state_dict(checkpoint['feature_standardizer'])
    self.target_standardizer = Standardizer.from_state_dict(checkpoint['target_standardizer'])
    self.target_mode = checkpoint.get('target_mode', 'absolute')
    self.window_sec = float(checkpoint['window_sec'])
    self.sample_rate_hz = float(checkpoint['sample_rate_hz'])
    self.sequence_len = int(np.floor(self.window_sec * self.sample_rate_hz)) + 1

    self.model = FutureTCNPredictor(**checkpoint['model_config']).to(device)
    self.model.load_state_dict(checkpoint['model_state'])
    self.model.eval()

  def _feature_at_times(self, history_rows, query_times):
    rows = sorted(history_rows, key=lambda row: row['time'])
    times = np.asarray([row['time'] for row in rows], dtype=np.float32)

    est_x = np.asarray([row.get('est_x', 0.0) for row in rows], dtype=np.float32)
    est_y = np.asarray([row.get('est_y', 0.0) for row in rows], dtype=np.float32)
    values = {
      'est_x': est_x,
      'est_y': est_y,
      'est_vx': np.asarray([row.get('est_vx', 0.0) for row in rows], dtype=np.float32),
      'est_vy': np.asarray([row.get('est_vy', 0.0) for row in rows], dtype=np.float32),
      'est_ax': np.asarray([row.get('est_ax', 0.0) for row in rows], dtype=np.float32),
      'est_ay': np.asarray([row.get('est_ay', 0.0) for row in rows], dtype=np.float32),
    }
    values['age_sec'] = query_times - query_times[-1]

    return np.stack(
      [
        values[column].astype(np.float32)
        if column == 'age_sec'
        else np.interp(query_times, times, values[column]).astype(np.float32)
        for column in self.feature_columns
      ],
      axis=-1)

  def predict(self, history_rows, horizon_sec):
    if len(history_rows) < 2:
      raise ValueError('Need at least two history rows for prediction.')

    current = max(history_rows, key=lambda row: row['time'])
    query_times = np.linspace(
      current['time'] - self.window_sec,
      current['time'],
      self.sequence_len)
    history_raw = self._feature_at_times(history_rows, query_times)
    est_x_idx = self.feature_columns.index('est_x')
    est_y_idx = self.feature_columns.index('est_y')
    current_position = history_raw[-1, [est_x_idx, est_y_idx]]
    history = self.feature_standardizer.transform(history_raw)

    with torch.no_grad():
      prediction_norm = self.model(
        torch.tensor(history, dtype=torch.float32, device=self.device).unsqueeze(0),
        torch.tensor([horizon_sec], dtype=torch.float32, device=self.device))

    prediction = self.target_standardizer.inverse_transform(prediction_norm.cpu().numpy())[0]
    if self.target_mode == 'delta':
      final_delta = prediction
      final_position = current_position + final_delta
    else:
      final_position = prediction
      final_delta = final_position - current_position
    return {
      'position': final_position,
      'delta': final_delta,
      'target_mode': self.target_mode,
    }
