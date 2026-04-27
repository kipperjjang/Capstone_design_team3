#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader

try:
  from ..dataset import PredictionWindowDataset, split_run_ids
  from .model import FutureGRUPredictor
except ImportError:
  sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
  from dataset import PredictionWindowDataset, split_run_ids
  from gru.model import FutureGRUPredictor


def parse_args():
  parser = argparse.ArgumentParser(description='Train GRU target future-position predictor.')
  parser.add_argument('--csv', nargs='+', required=True, help='CSV log files. Globs are expanded by the shell.')
  parser.add_argument('--out', default='checkpoints/target_predictor.pt', help='Output checkpoint path.')
  parser.add_argument('--horizon', type=float, default=0.5, help='Prediction horizon [s].')
  parser.add_argument('--window', type=float, default=0.8, help='Past history window [s].')
  parser.add_argument('--sample-rate', type=float, default=50.0, help='Resampled history rate [Hz].')
  parser.add_argument('--hidden-dim', type=int, default=64)
  parser.add_argument('--layers', type=int, default=1)
  parser.add_argument('--dropout', type=float, default=0.0)
  parser.add_argument('--batch-size', type=int, default=128)
  parser.add_argument('--epochs', type=int, default=100)
  parser.add_argument('--lr', type=float, default=1e-3)
  parser.add_argument('--val-fraction', type=float, default=0.2)
  parser.add_argument('--seed', type=int, default=0)
  parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
  return parser.parse_args()


def make_loader(dataset, batch_size, shuffle):
  return DataLoader(
    dataset,
    batch_size=batch_size,
    shuffle=shuffle,
    num_workers=0,
    drop_last=False)


def run_epoch(model, loader, optimizer, loss_fn, device, target_standardizer):
  training = optimizer is not None
  model.train(training)

  total_loss = 0.0
  total_count = 0
  total_position_sq_error = 0.0

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

    pred_position = prediction.detach().cpu()
    pred_position_px = torch.tensor(
      target_standardizer.inverse_transform(pred_position.numpy()),
      dtype=torch.float32)
    final_truth = batch['truth_future']
    total_position_sq_error += torch.sum(torch.sum((pred_position_px - final_truth) ** 2, dim=1)).item()

  mean_loss = total_loss / max(total_count, 1)
  rmse_px = (total_position_sq_error / max(total_count, 1)) ** 0.5
  return mean_loss, rmse_px


def main():
  args = parse_args()
  torch.manual_seed(args.seed)

  train_ids, val_ids = split_run_ids(args.csv, args.val_fraction, args.seed)
  print('train runs:', train_ids)
  print('val runs:', val_ids if val_ids else '(none)')

  train_dataset = PredictionWindowDataset(
    args.csv,
    horizon_sec=args.horizon,
    window_sec=args.window,
    sample_rate_hz=args.sample_rate,
    run_ids=train_ids)

  val_dataset = None
  if val_ids:
    val_dataset = PredictionWindowDataset(
      args.csv,
      horizon_sec=args.horizon,
      window_sec=args.window,
      sample_rate_hz=args.sample_rate,
      run_ids=val_ids,
      feature_standardizer=train_dataset.feature_standardizer,
      target_standardizer=train_dataset.target_standardizer)

  model = FutureGRUPredictor(
    input_dim=len(train_dataset.feature_columns),
    hidden_dim=args.hidden_dim,
    num_layers=args.layers,
    dropout=args.dropout).to(args.device)
  optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
  loss_fn = nn.HuberLoss(delta=1.0)

  train_loader = make_loader(train_dataset, args.batch_size, shuffle=True)
  val_loader = make_loader(val_dataset, args.batch_size, shuffle=False) if val_dataset else None

  best_score = float('inf')
  best_state = None
  for epoch in range(1, args.epochs + 1):
    train_loss, train_rmse = run_epoch(
      model,
      train_loader,
      optimizer,
      loss_fn,
      args.device,
      train_dataset.target_standardizer)

    if val_loader is not None:
      val_loss, val_rmse = run_epoch(
        model,
        val_loader,
        None,
        loss_fn,
        args.device,
        train_dataset.target_standardizer)
      score = val_rmse
      print(
        'epoch %03d train_loss=%.5f train_rmse=%.3fpx val_loss=%.5f val_rmse=%.3fpx' %
        (epoch, train_loss, train_rmse, val_loss, val_rmse))
    else:
      score = train_rmse
      print('epoch %03d train_loss=%.5f train_rmse=%.3fpx' % (epoch, train_loss, train_rmse))

    if score < best_score:
      best_score = score
      best_state = {key: value.detach().cpu() for key, value in model.state_dict().items()}

  out_path = Path(args.out)
  out_path.parent.mkdir(parents=True, exist_ok=True)
  torch.save({
    'model_state': best_state,
    'model_config': {
      'input_dim': len(train_dataset.feature_columns),
      'hidden_dim': args.hidden_dim,
      'num_layers': args.layers,
      'dropout': args.dropout,
    },
    'feature_columns': train_dataset.feature_columns,
    'feature_standardizer': train_dataset.feature_standardizer.state_dict(),
    'target_standardizer': train_dataset.target_standardizer.state_dict(),
    'horizon_sec': args.horizon,
    'window_sec': args.window,
    'sample_rate_hz': args.sample_rate,
    'model_type': 'gru',
    'prediction_type': 'absolute_future_position',
    'best_rmse_px': best_score,
  }, out_path)
  print('saved:', out_path)
  print('best_rmse_px:', best_score)


if __name__ == '__main__':
  main()
