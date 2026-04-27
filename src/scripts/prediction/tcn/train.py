#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader

try:
  from ..dataset import PredictionWindowDataset, split_run_ids
  from .model import FutureTCNPredictor
except ImportError:
  sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
  from dataset import PredictionWindowDataset, split_run_ids
  from tcn.model import FutureTCNPredictor


def parse_args():
  parser = argparse.ArgumentParser(description='Train TCN target future-position predictor.')
  parser.add_argument('--csv', nargs='+', required=True, help='CSV log files. Globs are expanded by the shell.')
  parser.add_argument('--out', default='checkpoints/tcn_target_predictor.pt', help='Output checkpoint path.')
  parser.add_argument('--horizon', type=float, default=None, help='Single prediction horizon [s]. Kept for compatibility.')
  parser.add_argument('--horizon-secs', type=float, nargs='+', default=None, help='Prediction horizons [s].')
  parser.add_argument('--target-mode', choices=['delta', 'absolute'], default='delta')
  parser.add_argument('--window', type=float, default=0.8, help='Past history window [s].')
  parser.add_argument('--sample-rate', type=float, default=50.0, help='Resampled history rate [Hz].')
  parser.add_argument('--channels', type=int, nargs='+', default=[64, 64, 64, 64], help='TCN channels per residual level.')
  parser.add_argument('--kernel-size', type=int, default=5)
  parser.add_argument('--dropout', type=float, default=0.1)
  parser.add_argument('--batch-size', type=int, default=128)
  parser.add_argument('--epochs', type=int, default=100)
  parser.add_argument('--lr', type=float, default=1e-3)
  parser.add_argument('--val-fraction', type=float, default=0.2)
  parser.add_argument('--seed', type=int, default=0)
  parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
  return parser.parse_args()


def resolve_horizon_secs(args):
  if args.horizon_secs is not None:
    values = args.horizon_secs
  elif args.horizon is not None:
    values = [args.horizon]
  else:
    values = [0.1, 0.2, 0.5, 1.0]
  values = [float(value) for value in values]
  if not values or any(value <= 0.0 for value in values):
    raise ValueError('Prediction horizons must be positive.')
  return values


def make_loader(dataset, batch_size, shuffle):
  return DataLoader(
    dataset,
    batch_size=batch_size,
    shuffle=shuffle,
    num_workers=0,
    drop_last=False)


def reconstruct_prediction(prediction_norm, batch, target_standardizer, target_mode):
  pred_target = target_standardizer.inverse_transform(prediction_norm.detach().cpu().numpy())
  pred_target = torch.tensor(pred_target, dtype=torch.float32)
  current_position = batch['current_position']
  truth_future = batch['truth_future']

  if target_mode == 'delta':
    pred_delta = pred_target
    pred_abs = current_position + pred_delta
  else:
    pred_abs = pred_target
    pred_delta = pred_abs - current_position

  return pred_delta, pred_abs, batch['target_delta'], truth_future


def run_epoch(model, loader, optimizer, loss_fn, device, target_standardizer, target_mode):
  training = optimizer is not None
  model.train(training)

  total_loss = 0.0
  total_count = 0
  total_delta_sq_error = 0.0
  total_abs_sq_error = 0.0
  total_delta_error = 0.0
  total_abs_error = 0.0

  for batch in loader:
    history = batch['history'].to(device)
    horizon = batch['horizon'].to(device)
    target = batch['target'].to(device)

    if training:
      optimizer.zero_grad()

    prediction = model(history, horizon)
    loss = loss_fn(prediction, target)

    if training:
      loss.backward()
      nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
      optimizer.step()

    batch_size = history.shape[0]
    total_loss += float(loss.item()) * batch_size
    total_count += batch_size

    pred_delta, pred_abs, target_delta, truth_future = reconstruct_prediction(
      prediction,
      batch,
      target_standardizer,
      target_mode)
    delta_norm = torch.linalg.norm(pred_delta - target_delta, dim=1)
    abs_norm = torch.linalg.norm(pred_abs - truth_future, dim=1)
    total_delta_sq_error += torch.sum(delta_norm ** 2).item()
    total_abs_sq_error += torch.sum(abs_norm ** 2).item()
    total_delta_error += torch.sum(delta_norm).item()
    total_abs_error += torch.sum(abs_norm).item()

  mean_loss = total_loss / max(total_count, 1)
  delta_rmse_px = (total_delta_sq_error / max(total_count, 1)) ** 0.5
  abs_rmse_px = (total_abs_sq_error / max(total_count, 1)) ** 0.5
  delta_mae_px = total_delta_error / max(total_count, 1)
  abs_mae_px = total_abs_error / max(total_count, 1)
  return mean_loss, delta_rmse_px, abs_rmse_px, delta_mae_px, abs_mae_px


def main():
  args = parse_args()
  torch.manual_seed(args.seed)
  horizon_secs = resolve_horizon_secs(args)
  max_horizon_sec = max(horizon_secs)

  train_ids, val_ids = split_run_ids(args.csv, args.val_fraction, args.seed)
  print('train runs:', train_ids)
  print('val runs:', val_ids if val_ids else '(none)')
  print('horizon_secs:', horizon_secs)
  print('target_mode:', args.target_mode)

  train_dataset = PredictionWindowDataset(
    args.csv,
    horizon_secs=horizon_secs,
    window_sec=args.window,
    sample_rate_hz=args.sample_rate,
    run_ids=train_ids,
    target_mode=args.target_mode)

  val_dataset = None
  if val_ids:
    val_dataset = PredictionWindowDataset(
      args.csv,
      horizon_secs=horizon_secs,
      window_sec=args.window,
      sample_rate_hz=args.sample_rate,
      run_ids=val_ids,
      feature_standardizer=train_dataset.feature_standardizer,
      target_standardizer=train_dataset.target_standardizer,
      target_mode=args.target_mode)

  model = FutureTCNPredictor(
    input_dim=len(train_dataset.feature_columns),
    channels=args.channels,
    kernel_size=args.kernel_size,
    dropout=args.dropout,
    max_horizon_sec=max_horizon_sec).to(args.device)
  optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
  loss_fn = nn.HuberLoss(delta=1.0)

  train_loader = make_loader(train_dataset, args.batch_size, shuffle=True)
  val_loader = make_loader(val_dataset, args.batch_size, shuffle=False) if val_dataset else None

  best_score = float('inf')
  best_state = None
  for epoch in range(1, args.epochs + 1):
    train_loss, train_delta_rmse, train_abs_rmse, train_delta_mae, train_abs_mae = run_epoch(
      model,
      train_loader,
      optimizer,
      loss_fn,
      args.device,
      train_dataset.target_standardizer,
      args.target_mode)

    if val_loader is not None:
      val_loss, val_delta_rmse, val_abs_rmse, val_delta_mae, val_abs_mae = run_epoch(
        model,
        val_loader,
        None,
        loss_fn,
        args.device,
        train_dataset.target_standardizer,
        args.target_mode)
      score = val_abs_rmse
      print(
        (
          'epoch %03d train_loss=%.5f train_delta_rmse=%.3fpx train_abs_rmse=%.3fpx '
          'train_delta_mae=%.3fpx train_abs_mae=%.3fpx val_loss=%.5f '
          'val_delta_rmse=%.3fpx val_abs_rmse=%.3fpx val_delta_mae=%.3fpx val_abs_mae=%.3fpx'
        ) %
        (
          epoch,
          train_loss,
          train_delta_rmse,
          train_abs_rmse,
          train_delta_mae,
          train_abs_mae,
          val_loss,
          val_delta_rmse,
          val_abs_rmse,
          val_delta_mae,
          val_abs_mae))
    else:
      score = train_abs_rmse
      print(
        (
          'epoch %03d train_loss=%.5f train_delta_rmse=%.3fpx train_abs_rmse=%.3fpx '
          'train_delta_mae=%.3fpx train_abs_mae=%.3fpx'
        ) %
        (epoch, train_loss, train_delta_rmse, train_abs_rmse, train_delta_mae, train_abs_mae))

    if score < best_score:
      best_score = score
      best_state = {key: value.detach().cpu() for key, value in model.state_dict().items()}

  out_path = Path(args.out)
  out_path.parent.mkdir(parents=True, exist_ok=True)
  torch.save({
    'model_state': best_state,
    'model_config': {
      'input_dim': len(train_dataset.feature_columns),
      'channels': list(args.channels),
      'kernel_size': args.kernel_size,
      'dropout': args.dropout,
      'max_horizon_sec': max_horizon_sec,
    },
    'feature_columns': train_dataset.feature_columns,
    'feature_standardizer': train_dataset.feature_standardizer.state_dict(),
    'target_standardizer': train_dataset.target_standardizer.state_dict(),
    'horizon_sec': horizon_secs[0],
    'horizon_secs': horizon_secs,
    'window_sec': args.window,
    'sample_rate_hz': args.sample_rate,
    'model_type': 'tcn',
    'target_mode': args.target_mode,
    'prediction_type': 'delta_future_position' if args.target_mode == 'delta' else 'absolute_future_position',
    'best_rmse_px': best_score,
    'best_abs_rmse_px': best_score,
  }, out_path)
  print('saved:', out_path)
  print('best_abs_rmse_px:', best_score)


if __name__ == '__main__':
  main()
