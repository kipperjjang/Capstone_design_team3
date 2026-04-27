#!/usr/bin/env python3
import torch
from torch import nn


class CausalConv1d(nn.Module):
  def __init__(self, in_channels, out_channels, kernel_size, dilation):
    super().__init__()
    self.padding = (kernel_size - 1) * dilation
    self.conv = nn.Conv1d(
      in_channels,
      out_channels,
      kernel_size=kernel_size,
      dilation=dilation,
      padding=self.padding)

  def forward(self, x):
    x = self.conv(x)
    if self.padding > 0:
      x = x[:, :, :-self.padding]
    return x


class TemporalBlock(nn.Module):
  def __init__(self, in_channels, out_channels, kernel_size, dilation, dropout):
    super().__init__()
    self.net = nn.Sequential(
      CausalConv1d(in_channels, out_channels, kernel_size, dilation),
      nn.ReLU(),
      nn.Dropout(dropout),
      CausalConv1d(out_channels, out_channels, kernel_size, dilation),
      nn.ReLU(),
      nn.Dropout(dropout))
    self.residual = (
      nn.Identity()
      if in_channels == out_channels
      else nn.Conv1d(in_channels, out_channels, kernel_size=1))
    self.out = nn.ReLU()

  def forward(self, x):
    return self.out(self.net(x) + self.residual(x))


class FutureTCNPredictor(nn.Module):
  """Predicts a future 2D target value from recent target history."""

  def __init__(self, input_dim, channels=None, kernel_size=5, dropout=0.1, max_horizon_sec=1.0):
    super().__init__()
    if channels is None:
      channels = [64, 64, 64, 64]
    if isinstance(channels, int):
      channels = [channels] * 4
    if len(channels) < 1:
      raise ValueError('channels must contain at least one level.')
    if kernel_size < 2:
      raise ValueError('kernel_size must be at least 2.')

    blocks = []
    in_channels = input_dim
    for level, out_channels in enumerate(channels):
      dilation = 2 ** level
      blocks.append(
        TemporalBlock(
          in_channels,
          int(out_channels),
          kernel_size=int(kernel_size),
          dilation=dilation,
          dropout=float(dropout)))
      in_channels = int(out_channels)

    self.max_horizon_sec = max(float(max_horizon_sec), 1e-6)
    self.tcn = nn.Sequential(*blocks)
    final_channels = int(channels[-1])
    head_hidden = max(32, final_channels)
    self.head = nn.Sequential(
      nn.Linear(final_channels * 2 + 1, head_hidden),
      nn.ReLU(),
      nn.Linear(head_hidden, head_hidden // 2),
      nn.ReLU(),
      nn.Linear(head_hidden // 2, 2))

  def forward(self, history, horizon):
    x = history.transpose(1, 2)
    features = self.tcn(x)
    last_features = features[:, :, -1]
    pooled_features = features.mean(dim=-1)
    if horizon.ndim == 1:
      horizon = horizon.unsqueeze(-1)
    horizon = horizon / self.max_horizon_sec
    return self.head(torch.cat([last_features, pooled_features, horizon], dim=-1))
