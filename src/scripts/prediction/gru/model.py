#!/usr/bin/env python3
import torch
from torch import nn


class FutureGRUPredictor(nn.Module):
  """Predicts the future 2D image position from recent target history."""

  def __init__(self, input_dim, hidden_dim=64, num_layers=1, dropout=0.0):
    super().__init__()
    gru_dropout = dropout if num_layers > 1 else 0.0
    self.gru = nn.GRU(
      input_size=input_dim,
      hidden_size=hidden_dim,
      num_layers=num_layers,
      batch_first=True,
      dropout=gru_dropout)
    self.head = nn.Sequential(
      nn.Linear(hidden_dim + 1, hidden_dim),
      nn.ReLU(),
      nn.Linear(hidden_dim, hidden_dim // 2),
      nn.ReLU(),
      nn.Linear(hidden_dim // 2, 2))

  def forward(self, history, horizon):
    _, hidden = self.gru(history)
    last_hidden = hidden[-1]
    if horizon.ndim == 1:
      horizon = horizon.unsqueeze(-1)
    return self.head(torch.cat([last_hidden, horizon], dim=-1))

