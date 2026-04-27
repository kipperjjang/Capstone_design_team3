#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import numpy as np


FIELDNAMES = [
  'run_id',
  'scenario_id',
  'time',
  'est_x',
  'est_y',
  'est_vx',
  'est_vy',
  'est_ax',
  'est_ay',
  'truth_x',
  'truth_y',
  'frequency_hz',
  'post_hit',
]


def parse_args():
  parser = argparse.ArgumentParser(
    description='Generate prediction training CSVs without running ROS or the Kalman filter.')
  parser.add_argument(
    '--output-csv',
    default='data/synthetic_training_samples.csv',
    help='Output CSV path.')
  parser.add_argument('--scenario-count', type=int, default=200)
  parser.add_argument('--periods-per-scenario', type=float, default=3.0)
  parser.add_argument('--sample-rate', type=float, default=30.0, help='Sample rate [Hz].')
  parser.add_argument('--random-seed', type=int, default=0, help='Use -1 for nondeterministic data.')
  parser.add_argument('--center-x', type=float, default=320.0)
  parser.add_argument('--center-y', type=float, default=240.0)
  parser.add_argument('--min-frequency-hz', type=float, default=0.25)
  parser.add_argument('--max-frequency-hz', type=float, default=1.20)
  parser.add_argument('--min-amplitude-x', type=float, default=5.0)
  parser.add_argument('--max-amplitude-x', type=float, default=140.0)
  parser.add_argument('--min-amplitude-y', type=float, default=40.0)
  parser.add_argument('--max-amplitude-y', type=float, default=170.0)
  parser.add_argument(
    '--state-source',
    choices=['truth', 'noisy_measurement'],
    default='truth',
    help='Source used for est_* input columns. truth is the clean simple case.')
  parser.add_argument(
    '--position-noise-std',
    type=float,
    default=0.0,
    help='Gaussian pixel noise added when --state-source noisy_measurement is used.')
  parser.add_argument(
    '--post-hit-switch',
    action='store_true',
    help='Enable the harder hidden trajectory switch used by the ROS/KF sampler.')
  return parser.parse_args()


def make_rng(seed):
  if seed < 0:
    return np.random.default_rng()
  return np.random.default_rng(seed)


def random_uniform(rng, low, high):
  return float(rng.uniform(low, high))


def make_sin_params(args, rng, direction_scale=1.0):
  base_frequency = random_uniform(rng, args.min_frequency_hz, args.max_frequency_hz)
  return {
    'frequency_hz': base_frequency,
    'amp_x1': random_uniform(rng, args.min_amplitude_x, args.max_amplitude_x) * direction_scale,
    'amp_y1': random_uniform(rng, args.min_amplitude_y, args.max_amplitude_y),
    'amp_x2': random_uniform(rng, 0.0, 0.35 * args.max_amplitude_x),
    'amp_y2': random_uniform(rng, 0.0, 0.35 * args.max_amplitude_y),
    'phase_x1': random_uniform(rng, 0.0, 2.0 * math.pi),
    'phase_y1': random_uniform(rng, 0.0, 2.0 * math.pi),
    'phase_x2': random_uniform(rng, 0.0, 2.0 * math.pi),
    'phase_y2': random_uniform(rng, 0.0, 2.0 * math.pi),
    'freq_ratio_x': random_uniform(rng, 0.75, 1.35),
    'freq_ratio_y': random_uniform(rng, 0.85, 1.25),
  }


def make_scenario(args, rng, scenario_id):
  pre = make_sin_params(args, rng, direction_scale=random_uniform(rng, 0.15, 0.75))
  post = make_sin_params(args, rng, direction_scale=random_uniform(rng, 0.65, 1.20))
  duration = max(0.1, args.periods_per_scenario / max(pre['frequency_hz'], 1e-6))
  hit_time = duration * random_uniform(rng, 0.45, 0.65)
  return {
    'id': scenario_id,
    'run_id': 'scenario_%04d' % scenario_id,
    'duration': duration,
    'hit_time': hit_time,
    'pre': pre,
    'post': post,
  }


def evaluate_params_state(args, params, t):
  wx = 2.0 * math.pi * params['frequency_hz'] * params['freq_ratio_x']
  wy = 2.0 * math.pi * params['frequency_hz'] * params['freq_ratio_y']
  position = np.array([
    args.center_x
    + params['amp_x1'] * math.sin(wx * t + params['phase_x1'])
    + params['amp_x2'] * math.sin(2.0 * wx * t + params['phase_x2']),
    args.center_y
    + params['amp_y1'] * math.sin(wy * t + params['phase_y1'])
    + params['amp_y2'] * math.sin(2.0 * wy * t + params['phase_y2']),
  ], dtype=np.float64)
  velocity = np.array([
    params['amp_x1'] * wx * math.cos(wx * t + params['phase_x1'])
    + params['amp_x2'] * 2.0 * wx * math.cos(2.0 * wx * t + params['phase_x2']),
    params['amp_y1'] * wy * math.cos(wy * t + params['phase_y1'])
    + params['amp_y2'] * 2.0 * wy * math.cos(2.0 * wy * t + params['phase_y2']),
  ], dtype=np.float64)
  acceleration = np.array([
    -params['amp_x1'] * wx * wx * math.sin(wx * t + params['phase_x1'])
    - params['amp_x2'] * 4.0 * wx * wx * math.sin(2.0 * wx * t + params['phase_x2']),
    -params['amp_y1'] * wy * wy * math.sin(wy * t + params['phase_y1'])
    - params['amp_y2'] * 4.0 * wy * wy * math.sin(2.0 * wy * t + params['phase_y2']),
  ], dtype=np.float64)
  return position, velocity, acceleration


def ground_truth_state(args, scenario, rel_time):
  if (not args.post_hit_switch) or rel_time < scenario['hit_time']:
    return evaluate_params_state(args, scenario['pre'], rel_time)

  hit_pos, _, _ = evaluate_params_state(args, scenario['pre'], scenario['hit_time'])
  post_zero, _, _ = evaluate_params_state(args, scenario['post'], 0.0)
  post_pos, post_vel, post_acc = evaluate_params_state(
    args,
    scenario['post'],
    rel_time - scenario['hit_time'])
  return hit_pos + (post_pos - post_zero), post_vel, post_acc


def sample_times(duration, sample_rate_hz):
  sample_rate_hz = max(float(sample_rate_hz), 1.0)
  count = int(math.floor(duration * sample_rate_hz)) + 1
  times = np.arange(count, dtype=np.float64) / sample_rate_hz
  if times.size == 0 or times[-1] < duration - 1e-9:
    times = np.append(times, duration)
  return times


def estimate_derivatives_causal(positions, times):
  velocities = np.zeros_like(positions)
  accelerations = np.zeros_like(positions)

  for i in range(1, len(times)):
    dt = times[i] - times[i - 1]
    if dt <= 0.0:
      velocities[i] = velocities[i - 1]
      accelerations[i] = accelerations[i - 1]
      continue

    velocities[i] = (positions[i] - positions[i - 1]) / dt
    accelerations[i] = (velocities[i] - velocities[i - 1]) / dt

  return velocities, accelerations


def write_scenario_rows(writer, args, rng, scenario):
  times = sample_times(scenario['duration'], args.sample_rate)
  truth_states = [ground_truth_state(args, scenario, float(t)) for t in times]
  truth_positions = np.stack([state[0] for state in truth_states], axis=0)
  truth_velocities = np.stack([state[1] for state in truth_states], axis=0)
  truth_accelerations = np.stack([state[2] for state in truth_states], axis=0)

  state_positions = truth_positions.copy()
  if args.state_source == 'truth':
    state_velocities = truth_velocities
    state_accelerations = truth_accelerations
  else:
    if args.position_noise_std > 0.0:
      state_positions += rng.normal(0.0, args.position_noise_std, size=state_positions.shape)
    state_velocities, state_accelerations = estimate_derivatives_causal(state_positions, times)

  for i, rel_time in enumerate(times):
    writer.writerow({
      'run_id': scenario['run_id'],
      'scenario_id': scenario['id'],
      'time': '%.9f' % rel_time,
      'est_x': '%.9f' % state_positions[i, 0],
      'est_y': '%.9f' % state_positions[i, 1],
      'est_vx': '%.9f' % state_velocities[i, 0],
      'est_vy': '%.9f' % state_velocities[i, 1],
      'est_ax': '%.9f' % state_accelerations[i, 0],
      'est_ay': '%.9f' % state_accelerations[i, 1],
      'truth_x': '%.9f' % truth_positions[i, 0],
      'truth_y': '%.9f' % truth_positions[i, 1],
      'frequency_hz': '%.9f' % scenario['pre']['frequency_hz'],
      'post_hit': int(args.post_hit_switch and rel_time >= scenario['hit_time']),
    })


def main():
  args = parse_args()
  rng = make_rng(args.random_seed)
  output_csv = Path(args.output_csv).expanduser()
  output_csv.parent.mkdir(parents=True, exist_ok=True)

  scenario_count = max(1, int(args.scenario_count))
  scenarios = [make_scenario(args, rng, i) for i in range(scenario_count)]

  with output_csv.open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
    writer.writeheader()
    for scenario in scenarios:
      write_scenario_rows(writer, args, rng, scenario)

  print('saved:', output_csv)
  print('scenario_count:', scenario_count)
  print('state_source:', args.state_source)
  print('post_hit_switch:', bool(args.post_hit_switch))


if __name__ == '__main__':
  main()
