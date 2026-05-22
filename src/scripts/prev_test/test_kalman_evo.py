#!/usr/bin/env python3
import math
import warnings

import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import TestDebug
from custom_msgs.msg import VisionMsg

import matplotlib.pyplot as plt

from evo.core import metrics as evo_metrics
from evo.core.trajectory import PoseTrajectory3D


class KalmanEvoVisualizer(Node):
  def __init__(self):
    super().__init__('test_kalman_evo')

    self.declare_parameter('debug_topic', '/test/debug')
    self.declare_parameter('vision_topic', '/vision')
    self.declare_parameter('sample_period_count', 2)
    self.declare_parameter('test_vision_rate_hz', 30.0)
    self.declare_parameter('test_center_x', 320.0)
    self.declare_parameter('test_center_y', 240.0)
    self.declare_parameter('test_amplitude_x', 120.0)
    self.declare_parameter('test_amplitude_y', 80.0)
    self.declare_parameter('test_frequency_hz', 0.5)
    self.declare_parameter('test_position_noise_std', 5.0)
    self.declare_parameter('random_seed', 0)
    self.declare_parameter('use_fixed_view', True)
    self.declare_parameter('view_min_x', 0.0)
    self.declare_parameter('view_max_x', 640.0)
    self.declare_parameter('view_min_y', 0.0)
    self.declare_parameter('view_max_y', 480.0)
    self.declare_parameter('debug_collection_grace_sec', 0.2)
    self.declare_parameter('evo_rpe_delta_frames', 1)
    self.declare_parameter('draw_prediction_included', True)

    self.debug_topic = self.get_parameter('debug_topic').value
    self.vision_topic = self.get_parameter('vision_topic').value
    self.sample_period_count = max(1, int(self.get_parameter('sample_period_count').value))
    self.test_vision_rate_hz = float(self.get_parameter('test_vision_rate_hz').value)
    self.test_center_x = float(self.get_parameter('test_center_x').value)
    self.test_center_y = float(self.get_parameter('test_center_y').value)
    self.test_amplitude_x = float(self.get_parameter('test_amplitude_x').value)
    self.test_amplitude_y = float(self.get_parameter('test_amplitude_y').value)
    self.test_frequency_hz = float(self.get_parameter('test_frequency_hz').value)
    self.test_position_noise_std = float(self.get_parameter('test_position_noise_std').value)
    self.random_seed = int(self.get_parameter('random_seed').value)
    self.use_fixed_view = bool(self.get_parameter('use_fixed_view').value)
    self.view_min_x = float(self.get_parameter('view_min_x').value)
    self.view_max_x = float(self.get_parameter('view_max_x').value)
    self.view_min_y = float(self.get_parameter('view_min_y').value)
    self.view_max_y = float(self.get_parameter('view_max_y').value)
    self.debug_collection_grace_sec = float(self.get_parameter('debug_collection_grace_sec').value)
    self.evo_rpe_delta_frames = max(1, int(self.get_parameter('evo_rpe_delta_frames').value))
    self.draw_prediction_included = bool(self.get_parameter('draw_prediction_included').value)

    self.period_sec = 1.0 / max(self.test_frequency_hz, 1e-6)
    self.sample_duration_sec = self.sample_period_count * self.period_sec
    self.rng = (
      np.random.default_rng()
      if self.random_seed < 0
      else np.random.default_rng(self.random_seed))

    self.raw_samples = []
    self.truth_samples = []
    self.kf_samples = []
    self.frozen_raw_samples = []
    self.frozen_truth_samples = []
    self.frozen_kf_samples = []

    self.test_vision_start_time = self.get_clock().now()
    self.synthetic_abs_origin = None
    self.last_test_position = None
    self.last_test_sample_time = None
    self.last_test_velocity = None
    self.last_published_stamp = None
    self.vision_done = False
    self.final_raw_debug_seen = False
    self.capture_frozen = False
    self.plots_rendered = False
    self.metric_report = {
      'ape': None,
      'rpe': None,
      'ape_times': [],
      'ape_values': [],
      'rpe_times': [],
      'rpe_values': [],
      'rpe_delta_time_sec': None,
      'error': None,
    }

    self.debug_sub = self.create_subscription(
      TestDebug,
      self.debug_topic,
      self.debugCallback,
      10)
    self.vision_pub = self.create_publisher(VisionMsg, self.vision_topic, 10)

    publish_period = 1.0 / max(self.test_vision_rate_hz, 1.0)
    self.test_vision_timer = self.create_timer(publish_period, self.publishTestVision)
    self.monitor_timer = self.create_timer(0.05, self.monitorAndRender)

    self.get_logger().info(
      'Sampling %.2f periods for %.3fs, then rendering final matplotlib plots.' %
      (self.sample_period_count, self.sample_duration_sec))

  def nowSeconds(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def groundTruthPosition(self, sample_time):
    phase = 2.0 * math.pi * self.test_frequency_hz * sample_time
    return np.array([
      self.test_center_x + self.test_amplitude_x * math.cos(phase)
      + self.test_amplitude_x/2 * math.sin(2*phase)
      + self.test_amplitude_x/3 * math.sin(3*phase),
      self.test_center_y + self.test_amplitude_y * math.sin(phase)
      + self.test_amplitude_y/2 * math.cos(2*phase)
      + self.test_amplitude_y/3 * math.cos(3*phase),
    ], dtype=float)

  def appendTruthSample(self, abs_time, rel_time, position):
    self.truth_samples.append((abs_time, rel_time, position))

  def appendRawSample(self, abs_time, rel_time, position, velocity):
    self.raw_samples.append((abs_time, rel_time, position, velocity))

  def appendKalmanSample(self, abs_time, rel_time, position, predicted_only):
    self.kf_samples.append((abs_time, rel_time, position, predicted_only))

  def publishTestVision(self):
    if self.vision_done:
      return

    now = self.get_clock().now()
    sample_time = (now - self.test_vision_start_time).nanoseconds * 1e-9
    stamp_time = now.nanoseconds * 1e-9
    if self.synthetic_abs_origin is None:
      self.synthetic_abs_origin = stamp_time - sample_time

    if sample_time > self.sample_duration_sec:
      self.stopSyntheticVision()
      return

    position = self.groundTruthPosition(sample_time)
    velocity = None
    acceleration = None
    if self.last_test_position is not None and self.last_test_sample_time is not None:
      dt = sample_time - self.last_test_sample_time
      if dt > 0.0:
        velocity = (position - self.last_test_position) / dt
        if self.last_test_velocity is not None:
          acceleration = (velocity - self.last_test_velocity) / dt

    observed_position = position.copy()
    if self.test_position_noise_std > 0.0:
      observed_position += self.rng.normal(0.0, self.test_position_noise_std, size=2)

    msg = VisionMsg()
    msg.header.stamp = now.to_msg()
    msg.detected = True
    msg.tracked = True
    msg.p = observed_position.tolist()
    msg.v = velocity.tolist() if velocity is not None else []
    msg.a = acceleration.tolist() if acceleration is not None else []
    msg.bbox = [0.0, 0.0]
    msg.confidence = 1.0
    msg.covariance = []
    msg.source_mode = 'test_kalman_evo'

    self.vision_pub.publish(msg)
    self.final_raw_debug_seen = False
    self.appendTruthSample(stamp_time, sample_time, position)
    self.appendRawSample(
      stamp_time,
      sample_time,
      observed_position,
      velocity if velocity is not None else np.zeros(2, dtype=float))

    self.last_test_position = position
    self.last_test_sample_time = sample_time
    self.last_test_velocity = velocity
    self.last_published_stamp = stamp_time

  def stopSyntheticVision(self):
    self.vision_done = True
    if self.test_vision_timer is not None:
      self.test_vision_timer.destroy()
      self.test_vision_timer = None
    self.get_logger().info(
      'Finished publishing synthetic vision. Waiting for final Kalman debug sample.')

  def debugCallback(self, msg):
    if self.capture_frozen or self.synthetic_abs_origin is None:
      return

    sample_time = float(msg.sample_time)
    rel_time = sample_time - self.synthetic_abs_origin

    if msg.estimator_initialized and -0.01 <= rel_time <= self.sample_duration_sec + 0.01:
      self.appendKalmanSample(
        sample_time,
        rel_time,
        np.array(msg.estimated_p, dtype=float),
        bool(msg.predicted_only))

    if (
      msg.has_raw
      and self.last_published_stamp is not None
      and sample_time >= self.last_published_stamp - 1e-6
    ):
      self.final_raw_debug_seen = True

  def maybeFreezeCapture(self):
    if self.capture_frozen or not self.vision_done:
      return

    waited_long_enough = False
    if self.last_published_stamp is not None:
      waited_long_enough = (
        self.nowSeconds() - self.last_published_stamp >= self.debug_collection_grace_sec)

    if self.final_raw_debug_seen or waited_long_enough:
      self.freezeCapture()

  def freezeCapture(self):
    self.capture_frozen = True
    self.frozen_raw_samples = [
      sample for sample in self.raw_samples
      if 0.0 <= sample[1] <= self.sample_duration_sec
    ]
    self.frozen_truth_samples = self.filteredTruthSamples()
    self.frozen_kf_samples = self.filteredKalmanSamples()
    self.computeEvoMetrics()
    self.get_logger().info(
      'Frozen capture with %d raw samples and %d Kalman samples.' %
      (len(self.frozen_raw_samples), len(self.frozen_kf_samples)))

  def filteredKalmanSamples(self):
    filtered = []
    last_rel_time = None
    for sample in sorted(self.kf_samples, key=lambda item: item[1]):
      rel_time = sample[1]
      if not 0.0 <= rel_time <= self.sample_duration_sec:
        continue
      if last_rel_time is not None and rel_time <= last_rel_time + 1e-6:
        continue
      filtered.append(sample)
      last_rel_time = rel_time
    return filtered

  def filteredTruthSamples(self):
    return [
      sample for sample in self.truth_samples
      if 0.0 <= sample[1] <= self.sample_duration_sec
    ]

  def estimationLabel(self):
    if self.draw_prediction_included:
      return 'prediction included estimation'
    return 'filtered estimation'

  def splitKalmanSamples(self, kf_samples):
    if self.draw_prediction_included:
      return list(kf_samples), []
    return [sample for sample in kf_samples if not sample[3]], []

  def kalmanSamplesForMetrics(self):
    if self.draw_prediction_included:
      # Use all estimator timestamps (including interpolated prediction-only points).
      return self.frozen_kf_samples
    return [sample for sample in self.frozen_kf_samples if not sample[3]]

  def makeEvoTrajectory(self, rel_times, positions):
    positions_xyz = np.zeros((len(positions), 3), dtype=float)
    positions_xyz[:, 0:2] = np.asarray(positions, dtype=float)
    orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=float), (len(positions), 1))
    return PoseTrajectory3D(
      positions_xyz=positions_xyz,
      orientations_quat_wxyz=orientations,
      timestamps=np.asarray(rel_times, dtype=float))

  def computeEvoMetrics(self):
    self.metric_report = {
      'ape': None,
      'rpe': None,
      'ape_times': [],
      'ape_values': [],
      'rpe_times': [],
      'rpe_values': [],
      'rpe_delta_time_sec': None,
      'error': None,
    }

    metric_samples = self.kalmanSamplesForMetrics()
    if not metric_samples:
      self.metric_report['error'] = 'Need at least 1 Kalman sample for error plotting.'
      return

    rel_times = []
    estimate_positions = []
    truth_positions = []
    for _, rel_time, estimate, _ in metric_samples:
      rel_times.append(rel_time)
      estimate_positions.append(estimate)
      # Evaluate the analytic trajectory at every estimator timestamp so
      # interpolated prediction-only points get a matching ground truth.
      truth_positions.append(self.groundTruthPosition(rel_time))

    estimate_arr = np.asarray(estimate_positions, dtype=float)
    truth_arr = np.asarray(truth_positions, dtype=float)
    times_arr = np.asarray(rel_times, dtype=float)

    ape_values = np.linalg.norm(estimate_arr - truth_arr, axis=1)
    self.metric_report['ape_times'] = times_arr.tolist()
    self.metric_report['ape_values'] = ape_values.tolist()

    if len(metric_samples) > self.evo_rpe_delta_frames:
      delta = self.evo_rpe_delta_frames
      delta_times = times_arr[delta:] - times_arr[:-delta]
      if len(delta_times) > 0:
        self.metric_report['rpe_delta_time_sec'] = float(np.median(delta_times))
      rpe_times = []
      rpe_values = []
      for i in range(delta, len(metric_samples)):
        est_delta = estimate_arr[i] - estimate_arr[i - delta]
        truth_delta = truth_arr[i] - truth_arr[i - delta]
        rpe_times.append(float(times_arr[i]))
        rpe_values.append(float(np.linalg.norm(est_delta - truth_delta)))
      self.metric_report['rpe_times'] = rpe_times
      self.metric_report['rpe_values'] = rpe_values

    if len(metric_samples) < 2:
      self.metric_report['error'] = 'Need at least 2 Kalman samples for APE/RPE.'
      return

    try:
      ref_traj = self.makeEvoTrajectory(rel_times, truth_positions)
      est_traj = self.makeEvoTrajectory(rel_times, estimate_positions)

      ape_metric = evo_metrics.APE(evo_metrics.PoseRelation.translation_part)
      ape_metric.process_data((ref_traj, est_traj))
      self.metric_report['ape'] = ape_metric.get_all_statistics()

      if len(metric_samples) > self.evo_rpe_delta_frames:
        rpe_metric = evo_metrics.RPE(
          evo_metrics.PoseRelation.translation_part,
          delta=self.evo_rpe_delta_frames,
          delta_unit=evo_metrics.Unit.frames,
          all_pairs=False)
        rpe_metric.process_data((ref_traj, est_traj))
        self.metric_report['rpe'] = rpe_metric.get_all_statistics()
    except Exception as exc:
      self.metric_report['error'] = '%s: %s' % (type(exc).__name__, exc)

  def toTimePointArrays(self, samples):
    if not samples:
      return np.array([], dtype=float), np.empty((0, 2), dtype=float)
    times = np.asarray([sample[1] for sample in samples], dtype=float)
    points = np.asarray([sample[2] for sample in samples], dtype=float)
    return times, points

  def getMetricRmse(self, metric_name):
    stats = self.metric_report.get(metric_name)
    if stats is not None and np.isfinite(stats.get('rmse', float('nan'))):
      return float(stats.get('rmse'))

    values = self.metric_report.get('%s_values' % metric_name, [])
    if values:
      arr = np.asarray(values, dtype=float)
      return float(np.sqrt(np.mean(arr * arr)))
    return None

  def periodColor(self, period_idx):
    colors = plt.rcParams['axes.prop_cycle'].by_key().get('color', [])
    if not colors:
      colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
    return colors[period_idx % len(colors)]

  def splitArraysByPeriod(self, times, values):
    if len(times) == 0:
      return []

    segments = []
    for period_idx in range(self.sample_period_count):
      start = period_idx * self.period_sec
      end = (period_idx + 1) * self.period_sec
      if period_idx == self.sample_period_count - 1:
        mask = (times >= start) & (times <= end + 1e-9)
      else:
        mask = (times >= start) & (times < end)
      if np.any(mask):
        segments.append((period_idx, times[mask], values[mask]))
    return segments

  def plotPeriodLine(self, ax, times, values, axis_idx=None, linestyle='-', linewidth=2,
                     alpha=1.0, label_prefix=None):
    for period_idx, period_times, period_values in self.splitArraysByPeriod(times, values):
      y_values = period_values[:, axis_idx] if axis_idx is not None else period_values
      label = None
      if label_prefix is not None:
        label = '%s period %d' % (label_prefix, period_idx + 1)
      ax.plot(
        period_times,
        y_values,
        linestyle=linestyle,
        color=self.periodColor(period_idx),
        linewidth=linewidth,
        alpha=alpha,
        label=label)

  def plotPeriodScatter(self, ax, times, values, axis_idx=None, s=18, marker='o',
                        alpha=1.0, label_prefix=None):
    for period_idx, period_times, period_values in self.splitArraysByPeriod(times, values):
      y_values = period_values[:, axis_idx] if axis_idx is not None else period_values
      label = None
      if label_prefix is not None:
        label = '%s period %d' % (label_prefix, period_idx + 1)
      ax.scatter(
        period_times,
        y_values,
        s=s,
        marker=marker,
        color=self.periodColor(period_idx),
        alpha=alpha,
        label=label)

  def plotPeriodTrajectory(self, ax, times, points, linestyle='-', linewidth=2,
                           alpha=1.0, marker=None, s=18, label_prefix=None):
    for period_idx, _, period_points in self.splitArraysByPeriod(times, points):
      label = None
      if label_prefix is not None:
        label = '%s period %d' % (label_prefix, period_idx + 1)
      if marker is None:
        ax.plot(
          period_points[:, 0],
          period_points[:, 1],
          linestyle=linestyle,
          color=self.periodColor(period_idx),
          linewidth=linewidth,
          alpha=alpha,
          label=label)
      else:
        ax.scatter(
          period_points[:, 0],
          period_points[:, 1],
          s=s,
          marker=marker,
          color=self.periodColor(period_idx),
          alpha=alpha,
          label=label)

  def plotTrajectoryFigure(self, raw_samples, filtered_samples, prediction_samples, truth_samples):
    truth_t, truth_xy = self.toTimePointArrays(truth_samples)
    raw_t, raw_xy = self.toTimePointArrays(raw_samples)
    filt_t, filt_xy = self.toTimePointArrays(filtered_samples)
    pred_t, pred_xy = self.toTimePointArrays(prediction_samples)

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    fig.subplots_adjust(top=0.72, wspace=0.28)
    ax_xy, ax_x, ax_y = axes

    if len(truth_xy) > 0:
      self.plotPeriodTrajectory(
        ax_xy,
        truth_t,
        truth_xy,
        linestyle='--',
        linewidth=2,
        alpha=0.8,
        label_prefix='ground truth')
    if len(raw_xy) > 0:
      self.plotPeriodTrajectory(
        ax_xy,
        raw_t,
        raw_xy,
        marker='o',
        s=18,
        alpha=0.42,
        label_prefix='noised samples')
    if len(filt_xy) > 0:
      self.plotPeriodTrajectory(
        ax_xy,
        filt_t,
        filt_xy,
        linewidth=2.4,
        label_prefix=self.estimationLabel())
    if len(pred_xy) > 0:
      self.plotPeriodTrajectory(
        ax_xy,
        pred_t,
        pred_xy,
        linewidth=2,
        alpha=0.7,
        label_prefix='prediction')
    ax_xy.scatter(
      [truth_xy[0, 0], truth_xy[-1, 0]],
      [truth_xy[0, 1], truth_xy[-1, 1]],
      marker='x',
      s=90,
      color='black',
      linewidths=2,
      label='start/end')
    if self.use_fixed_view:
      ax_xy.set_xlim(self.view_min_x, self.view_max_x)
      ax_xy.set_ylim(self.view_min_y, self.view_max_y)
    ax_xy.set_xlabel('x [px]')
    ax_xy.set_ylabel('y [px]')
    ax_xy.set_title('Trajectory (x-y)')
    ax_xy.grid(True, alpha=0.3)

    if len(truth_xy) > 0:
      self.plotPeriodLine(
        ax_x,
        truth_t,
        truth_xy,
        axis_idx=0,
        linestyle='--',
        linewidth=2,
        alpha=0.8)
      ax_x.scatter([truth_t[0], truth_t[-1]], [truth_xy[0, 0], truth_xy[-1, 0]],
                   marker='x', s=90, color='black', linewidths=2)
    if len(raw_xy) > 0:
      self.plotPeriodScatter(ax_x, raw_t, raw_xy, axis_idx=0, s=18, alpha=0.42)
    if len(filt_xy) > 0:
      self.plotPeriodLine(ax_x, filt_t, filt_xy, axis_idx=0, linewidth=2.4)
    if len(pred_xy) > 0:
      self.plotPeriodLine(ax_x, pred_t, pred_xy, axis_idx=0, linewidth=2, alpha=0.7)
    ax_x.set_xlabel('time [s]')
    ax_x.set_ylabel('x [px]')
    ax_x.set_title('x over time')
    ax_x.grid(True, alpha=0.3)

    if len(truth_xy) > 0:
      self.plotPeriodLine(
        ax_y,
        truth_t,
        truth_xy,
        axis_idx=1,
        linestyle='--',
        linewidth=2,
        alpha=0.8)
      ax_y.scatter([truth_t[0], truth_t[-1]], [truth_xy[0, 1], truth_xy[-1, 1]],
                   marker='x', s=90, color='black', linewidths=2)
    if len(raw_xy) > 0:
      self.plotPeriodScatter(ax_y, raw_t, raw_xy, axis_idx=1, s=18, alpha=0.42)
    if len(filt_xy) > 0:
      self.plotPeriodLine(ax_y, filt_t, filt_xy, axis_idx=1, linewidth=2.4)
    if len(pred_xy) > 0:
      self.plotPeriodLine(ax_y, pred_t, pred_xy, axis_idx=1, linewidth=2, alpha=0.7)
    ax_y.set_xlabel('time [s]')
    ax_y.set_ylabel('y [px]')
    ax_y.set_title('y over time')
    ax_y.grid(True, alpha=0.3)

    handles, labels = ax_xy.get_legend_handles_labels()
    fig.legend(
      handles,
      labels,
      ncol=min(len(labels), 4),
      loc='upper center',
      bbox_to_anchor=(0.5, 0.91),
      frameon=False)

    fig.suptitle('Kalman Filter Trajectory Summary', fontsize=14, y=0.99)
    return fig

  def plotErrorFigure(self):
    ape_times = np.asarray(self.metric_report.get('ape_times', []), dtype=float)
    ape_values = np.asarray(self.metric_report.get('ape_values', []), dtype=float)
    rpe_times = np.asarray(self.metric_report.get('rpe_times', []), dtype=float)
    rpe_values = np.asarray(self.metric_report.get('rpe_values', []), dtype=float)

    fig, ax = plt.subplots(figsize=(10.5, 4.8), constrained_layout=True)
    if len(ape_times) > 0:
      ax.plot(ape_times, ape_values, color='tab:blue', linewidth=2, label='APE')
    if len(rpe_times) > 0:
      ax.plot(rpe_times, rpe_values, color='tab:red', linewidth=2, label='RPE')

    ax.set_xlabel('time [s]')
    ax.set_ylabel('error [px]')
    rpe_delta_time = self.metric_report.get('rpe_delta_time_sec')
    if rpe_delta_time is not None:
      ax.set_title(
        'APE / RPE over time (RPE %d frame(s) = %.4fs)' %
        (self.evo_rpe_delta_frames, rpe_delta_time))
    else:
      ax.set_title('APE / RPE over time')
    ax.set_xlim(left=0.0, right=max(self.sample_duration_sec, 1e-6))
    ax.set_ylim(bottom=0.0)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')

    ape_rmse = self.getMetricRmse('ape')
    rpe_rmse = self.getMetricRmse('rpe')
    if rpe_delta_time is not None:
      rpe_delta_text = '%d frame(s) = %.4f s' % (
        self.evo_rpe_delta_frames,
        rpe_delta_time)
    else:
      rpe_delta_text = '%d frame(s)' % self.evo_rpe_delta_frames
    metrics_text = '\n'.join([
      'Overall APE RMSE: %s' % ('%.3f px' % ape_rmse if ape_rmse is not None else 'n/a'),
      'Overall RPE RMSE: %s' % ('%.3f px' % rpe_rmse if rpe_rmse is not None else 'n/a'),
      'RPE delta: %s' % rpe_delta_text,
    ])
    ax.text(
      0.98,
      0.98,
      metrics_text,
      transform=ax.transAxes,
      va='top',
      ha='right',
      fontsize=10,
      bbox=dict(facecolor='white', edgecolor='0.85', boxstyle='round,pad=0.4'))

    error_note = self.metric_report.get('error')
    if error_note:
      fig.text(0.01, 0.01, error_note, fontsize=9, color='0.35')

    return fig

  def showFinalPlots(self):
    raw_samples = self.frozen_raw_samples
    truth_samples = self.frozen_truth_samples
    filtered_samples, prediction_samples = self.splitKalmanSamples(self.frozen_kf_samples)

    self.plotTrajectoryFigure(raw_samples, filtered_samples, prediction_samples, truth_samples)
    self.plotErrorFigure()
    plt.show()

  def monitorAndRender(self):
    self.maybeFreezeCapture()

    if not self.capture_frozen or self.plots_rendered:
      return

    self.plots_rendered = True
    self.showFinalPlots()
    rclpy.shutdown()


def main():
  rclpy.init()
  node = KalmanEvoVisualizer()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()
