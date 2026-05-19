#!/usr/bin/env python3
"""Checkerboard camera calibration utilities.

This file can be imported as a small module or run directly as a CLI.

Example:
  python3 camera_calibration.py images \
    --input ./calib_images \
    --board-cols 9 --board-rows 6 \
    --square-size 0.025 \
    --output camera_calibration.yaml

The board size is the number of inner corners, not the number of squares.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


ImageSize = Tuple[int, int]
BoardSize = Tuple[int, int]


def jetson_gstreamer_pipeline(
    sensor_id: int = 0,
    capture_width: int = 1280,
    capture_height: int = 720,
    display_width: int = 640,
    display_height: int = 480,
    framerate: int = 30,
    flip_method: int = 0) -> str:
  return (
    f'nvarguscamerasrc sensor-id={sensor_id} ! '
    f'video/x-raw(memory:NVMM), '
    f'width=(int){capture_width}, '
    f'height=(int){capture_height}, '
    f'framerate=(fraction){framerate}/1 ! '
    f'nvvidconv flip-method={flip_method} ! '
    f'video/x-raw, '
    f'width=(int){display_width}, '
    f'height=(int){display_height}, '
    f'format=(string)BGRx ! '
    f'videoconvert ! '
    f'video/x-raw, format=(string)BGR ! '
    f'appsink drop=true max-buffers=1 sync=false'
  )


@dataclass
class CalibrationResult:
  camera_matrix: np.ndarray
  dist_coeffs: np.ndarray
  rvecs: Sequence[np.ndarray]
  tvecs: Sequence[np.ndarray]
  image_size: ImageSize
  reprojection_error: float
  used_images: Sequence[str]

  def undistort(self, image: np.ndarray, crop: bool = False) -> np.ndarray:
    """Return an undistorted copy of image using this calibration."""
    h, w = image.shape[:2]
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(
      self.camera_matrix,
      self.dist_coeffs,
      (w, h),
      1,
      (w, h))
    undistorted = cv2.undistort(
      image,
      self.camera_matrix,
      self.dist_coeffs,
      None,
      new_matrix)

    if crop:
      x, y, roi_w, roi_h = roi
      if roi_w > 0 and roi_h > 0:
        undistorted = undistorted[y:y + roi_h, x:x + roi_w]
    return undistorted


class ChessboardCalibrator:
  def __init__(
      self,
      board_size: BoardSize = (9, 6),
      square_size: float = 1.0,
      criteria: Optional[Tuple[int, int, float]] = None):
    if board_size[0] < 2 or board_size[1] < 2:
      raise ValueError('board_size must be inner corners, e.g. (9, 6).')
    if square_size <= 0.0:
      raise ValueError('square_size must be positive.')

    self.board_size = board_size
    self.square_size = float(square_size)
    self.criteria = criteria or (
      cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
      30,
      0.001)
    self.object_template = self._make_object_points()

  def _make_object_points(self) -> np.ndarray:
    cols, rows = self.board_size
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= self.square_size
    return points

  def find_corners(
      self,
      image: np.ndarray,
      refine: bool = True) -> Tuple[bool, Optional[np.ndarray]]:
    gray = self._to_gray(image)

    if hasattr(cv2, 'findChessboardCornersSB'):
      sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE
      found, corners = cv2.findChessboardCornersSB(gray, self.board_size, sb_flags)
      if found:
        return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, self.board_size, flags)
    if not found:
      return False, None

    if refine:
      corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        self.criteria)
    return True, corners

  def collect_points(
      self,
      images: Iterable[Tuple[str, np.ndarray]],
      preview_dir: Optional[Path] = None) -> Tuple[List[np.ndarray], List[np.ndarray], ImageSize, List[str]]:
    object_points: List[np.ndarray] = []
    image_points: List[np.ndarray] = []
    used_images: List[str] = []
    image_size: Optional[ImageSize] = None

    if preview_dir is not None:
      preview_dir.mkdir(parents=True, exist_ok=True)

    for name, image in images:
      if image is None:
        continue
      h, w = image.shape[:2]
      if image_size is None:
        image_size = (w, h)
      elif image_size != (w, h):
        raise ValueError(
          'All calibration images must have the same size. '
          f'Expected {image_size}, got {(w, h)} from {name}.')

      found, corners = self.find_corners(image)
      if not found or corners is None:
        print(f'[skip] checkerboard not found: {name}')
        continue

      object_points.append(self.object_template.copy())
      image_points.append(corners)
      used_images.append(name)
      print(f'[ok] checkerboard found: {name}')

      if preview_dir is not None:
        preview = image.copy()
        cv2.drawChessboardCorners(preview, self.board_size, corners, found)
        cv2.imwrite(str(preview_dir / f'{Path(name).stem}_corners.jpg'), preview)

    if image_size is None:
      raise RuntimeError('No readable images were provided.')
    return object_points, image_points, image_size, used_images

  def calibrate(
      self,
      images: Iterable[Tuple[str, np.ndarray]],
      preview_dir: Optional[Path] = None,
      min_images: int = 10) -> CalibrationResult:
    object_points, image_points, image_size, used_images = self.collect_points(
      images,
      preview_dir=preview_dir)

    if len(image_points) < min_images:
      raise RuntimeError(
        f'Need at least {min_images} valid checkerboard images, '
        f'but only found {len(image_points)}.')

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
      object_points,
      image_points,
      image_size,
      None,
      None)

    mean_error = self.reprojection_error(
      object_points,
      image_points,
      rvecs,
      tvecs,
      camera_matrix,
      dist_coeffs)

    return CalibrationResult(
      camera_matrix=camera_matrix,
      dist_coeffs=dist_coeffs,
      rvecs=rvecs,
      tvecs=tvecs,
      image_size=image_size,
      reprojection_error=float(mean_error if mean_error > 0.0 else rms),
      used_images=used_images)

  @staticmethod
  def reprojection_error(
      object_points: Sequence[np.ndarray],
      image_points: Sequence[np.ndarray],
      rvecs: Sequence[np.ndarray],
      tvecs: Sequence[np.ndarray],
      camera_matrix: np.ndarray,
      dist_coeffs: np.ndarray) -> float:
    total_error = 0.0
    total_points = 0

    for objp, imgp, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
      projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
      error = cv2.norm(imgp, projected, cv2.NORM_L2)
      total_error += error * error
      total_points += len(objp)

    if total_points == 0:
      return 0.0
    return float(np.sqrt(total_error / total_points))

  @staticmethod
  def _to_gray(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
      return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def iter_image_files(input_path: Path) -> Iterable[Tuple[str, np.ndarray]]:
  patterns = ('*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tif', '*.tiff')
  files: List[Path] = []

  if input_path.is_file():
    files = [input_path]
  else:
    for pattern in patterns:
      files.extend(sorted(input_path.glob(pattern)))

  for path in files:
    temp = cv2.imread(str(path), cv2.IMREAD_COLOR)
    yield str(path), cv2.rotate(temp, cv2.ROTATE_180)


def open_video_capture(
    source: str,
    width: Optional[int] = None,
    height: Optional[int] = None,
    fps: Optional[int] = None,
    camera_type: str = 'webcam',
    sensor_id: int = 0,
    capture_width: Optional[int] = None,
    capture_height: Optional[int] = None,
    flip_method: int = 0) -> cv2.VideoCapture:
  """Open camera/video source.

  webcam: numeric sources use V4L2/MJPG like vision.py's WebCAMLatestFrameReader.
  csi: Jetson IMX477/CSI sources use vision.py's nvarguscamerasrc pipeline.
  file: file paths are opened with OpenCV's default backend.
  """
  if camera_type == 'csi':
    display_width = width or capture_width or 1280
    display_height = height or capture_height or 720
    pipeline = jetson_gstreamer_pipeline(
      sensor_id=sensor_id,
      capture_width=capture_width or display_width,
      capture_height=capture_height or display_height,
      display_width=display_width,
      display_height=display_height,
      framerate=fps or 30,
      flip_method=flip_method)
    print('GStreamer pipeline:', pipeline)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
  elif camera_type == 'webcam' and source.isdigit():
    cap = cv2.VideoCapture(int(source), cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    if width:
      cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
      cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
      cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
  else:
    cap = cv2.VideoCapture(source)

  return cap


class WebcamStream:
  def __init__(
      self,
      source: str = '1',
      width: Optional[int] = 1280,
      height: Optional[int] = 720,
      fps: Optional[int] = 30,
      camera_type: str = 'webcam',
      sensor_id: int = 0,
      capture_width: Optional[int] = None,
      capture_height: Optional[int] = None,
      flip_method: int = 0):
    self.source = source
    self.cap = open_video_capture(
      source,
      width=width,
      height=height,
      fps=fps,
      camera_type=camera_type,
      sensor_id=sensor_id,
      capture_width=capture_width,
      capture_height=capture_height,
      flip_method=flip_method)

    if not self.cap.isOpened():
      raise RuntimeError(
        f'Camera source {source} could not be opened. '
        'Try --source 0, --source 1, or check v4l2-ctl --list-devices.')

    print('Actual width :', self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print('Actual height:', self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print('Actual FPS   :', self.cap.get(cv2.CAP_PROP_FPS))

    self.ret, self.frame = self.cap.read()
    if not self.ret:
      self.cap.release()
      raise RuntimeError('Camera opened, but first frame could not be read.')

    self.running = True
    self.lock = threading.Lock()
    self.thread = threading.Thread(target=self.update, daemon=True)
    self.thread.start()

  def update(self) -> None:
    while self.running:
      ret, frame = self.cap.read()
      if ret:
        with self.lock:
          self.ret = ret
          self.frame = frame
      else:
        time.sleep(0.001)

  def read(self) -> Tuple[bool, Optional[np.ndarray]]:
    with self.lock:
      if self.frame is None:
        return False, None
      return self.ret, self.frame.copy()

  def stop(self) -> None:
    self.running = False
    self.thread.join(timeout=1.0)
    self.cap.release()


def iter_video_frames(
    source: str,
    every_n: int = 15,
    max_frames: int = 80,
    camera_width: Optional[int] = None,
    camera_height: Optional[int] = None,
    camera_fps: Optional[int] = None,
    camera_type: str = 'webcam',
    sensor_id: int = 0,
    capture_width: Optional[int] = None,
    capture_height: Optional[int] = None,
    flip_method: int = 0) -> Iterable[Tuple[str, np.ndarray]]:
  cap = open_video_capture(
    source,
    width=camera_width,
    height=camera_height,
    fps=camera_fps,
    camera_type=camera_type,
    sensor_id=sensor_id,
    capture_width=capture_width,
    capture_height=capture_height,
    flip_method=flip_method)
  if not cap.isOpened():
    raise RuntimeError(f'Could not open video/camera source: {source}')

  frame_index = 0
  used = 0
  try:
    while used < max_frames:
      ok, frame = cap.read()
      if not ok:
        break
      if frame_index % every_n == 0:
        yield f'{source}:frame_{frame_index:06d}', frame
        used += 1
      frame_index += 1
  finally:
    cap.release()


def collect_checkerboard_images(
    source: str,
    output_dir: Path,
    board_size: BoardSize = (9, 6),
    square_size: float = 1.0,
    camera_width: Optional[int] = None,
    camera_height: Optional[int] = None,
    camera_fps: Optional[int] = None,
    camera_type: str = 'webcam',
    sensor_id: int = 0,
    capture_width: Optional[int] = None,
    capture_height: Optional[int] = None,
    flip_method: int = 0,
    required_count: int = 20) -> None:
  """Interactively capture checkerboard frames from a camera.

  Press Space to save a frame when corners are detected, q/Esc to quit.
  """
  output_dir.mkdir(parents=True, exist_ok=True)
  stream = WebcamStream(
    source,
    width=camera_width,
    height=camera_height,
    fps=camera_fps,
    camera_type=camera_type,
    sensor_id=sensor_id,
    capture_width=capture_width,
    capture_height=capture_height,
    flip_method=flip_method)

  calibrator = ChessboardCalibrator(board_size, square_size)
  detection_lock = threading.Lock()
  detection_running = True
  detection_found = False
  detection_corners = None
  detection_frame = None
  detection_time = 0.0

  def detect_loop() -> None:
    nonlocal detection_found
    nonlocal detection_corners
    nonlocal detection_frame
    nonlocal detection_time

    while detection_running:
      ok, frame = stream.read()
      if not ok or frame is None:
        time.sleep(0.001)
        continue

      found, corners = calibrator.find_corners(frame)
      with detection_lock:
        detection_found = found
        detection_corners = corners
        detection_frame = frame
        detection_time = time.time()

  detector_thread = threading.Thread(target=detect_loop, daemon=True)
  detector_thread.start()

  saved = 0
  prev_time = time.time()
  fps_counter = 0
  display_fps = 0
  try:
    while saved < required_count:
      ok, frame = stream.read()
      if not ok or frame is None:
        continue

      fps_counter += 1
      now = time.time()
      if now - prev_time >= 1.0:
        display_fps = fps_counter
        fps_counter = 0
        prev_time = now

      preview = frame.copy()
      with detection_lock:
        found = detection_found
        corners = None if detection_corners is None else detection_corners.copy()
        frame_to_save = None if detection_frame is None else detection_frame.copy()
        detection_age = now - detection_time if detection_time > 0.0 else 999.0

      if found and corners is not None and detection_age < 0.5:
        cv2.drawChessboardCorners(preview, board_size, corners, found)
      cv2.putText(
        preview,
        f'saved {saved}/{required_count} | FPS {display_fps} | Space/s: save | q/Esc: quit',
        (18, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0) if found and detection_age < 0.5 else (0, 0, 255),
        2,
        cv2.LINE_AA)
      cv2.imshow('checkerboard_capture', preview)

      key = cv2.waitKey(1) & 0xFF
      if key in (ord('q'), 27):
        break
      if key in (ord(' '), ord('s')):
        if found and frame_to_save is not None and detection_age < 0.5:
          save_frame = frame_to_save
        else:
          save_frame = frame
          print('[warn] saving current frame without a fresh checkerboard detection')

        path = output_dir / f'calib_{saved:03d}.jpg'
        cv2.imwrite(str(path), save_frame)
        print(f'[save] {path}')
        saved += 1
  finally:
    detection_running = False
    detector_thread.join(timeout=1.0)
    stream.stop()
    cv2.destroyAllWindows()


def calibrate_from_images(
    input_path: str,
    board_size: BoardSize = (9, 6),
    square_size: float = 1.0,
    output_path: Optional[str] = None,
    preview_dir: Optional[str] = None,
    min_images: int = 10) -> CalibrationResult:
  calibrator = ChessboardCalibrator(board_size, square_size)
  result = calibrator.calibrate(
    iter_image_files(Path(input_path)),
    preview_dir=Path(preview_dir) if preview_dir else None,
    min_images=min_images)
  if output_path:
    save_calibration(result, output_path, board_size, square_size)
  return result


def calibrate_from_video(
    source: str,
    board_size: BoardSize = (9, 6),
    square_size: float = 1.0,
    output_path: Optional[str] = None,
    preview_dir: Optional[str] = None,
    every_n: int = 15,
    max_frames: int = 80,
    camera_width: Optional[int] = None,
    camera_height: Optional[int] = None,
    camera_fps: Optional[int] = None,
    camera_type: str = 'webcam',
    sensor_id: int = 0,
    capture_width: Optional[int] = None,
    capture_height: Optional[int] = None,
    flip_method: int = 0,
    min_images: int = 10) -> CalibrationResult:
  calibrator = ChessboardCalibrator(board_size, square_size)
  result = calibrator.calibrate(
    iter_video_frames(
      source,
      every_n=every_n,
      max_frames=max_frames,
      camera_width=camera_width,
      camera_height=camera_height,
      camera_fps=camera_fps,
      camera_type=camera_type,
      sensor_id=sensor_id,
      capture_width=capture_width,
      capture_height=capture_height,
      flip_method=flip_method),
    preview_dir=Path(preview_dir) if preview_dir else None,
    min_images=min_images)
  if output_path:
    save_calibration(result, output_path, board_size, square_size)
  return result


def save_calibration(
    result: CalibrationResult,
    output_path: str,
    board_size: BoardSize = (9, 6),
    square_size: float = 1.0) -> None:
  path = Path(output_path)
  path.parent.mkdir(parents=True, exist_ok=True)

  fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
  if not fs.isOpened():
    raise RuntimeError(f'Could not write calibration file: {path}')
  try:
    fs.write('image_width', int(result.image_size[0]))
    fs.write('image_height', int(result.image_size[1]))
    fs.write('board_cols', int(board_size[0]))
    fs.write('board_rows', int(board_size[1]))
    fs.write('square_size', float(square_size))
    fs.write('reprojection_error', float(result.reprojection_error))
    fs.write('camera_matrix', result.camera_matrix)
    fs.write('distortion_coefficients', result.dist_coeffs)
  finally:
    fs.release()


def load_calibration(path: str) -> CalibrationResult:
  fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
  if not fs.isOpened():
    raise RuntimeError(f'Could not read calibration file: {path}')
  try:
    width = int(fs.getNode('image_width').real())
    height = int(fs.getNode('image_height').real())
    error = float(fs.getNode('reprojection_error').real())
    camera_matrix = fs.getNode('camera_matrix').mat()
    dist_coeffs = fs.getNode('distortion_coefficients').mat()
  finally:
    fs.release()

  return CalibrationResult(
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    rvecs=[],
    tvecs=[],
    image_size=(width, height),
    reprojection_error=error,
    used_images=[])


def parse_board_size(text: str) -> BoardSize:
  normalized = text.lower().replace('x', ',')
  parts = [part.strip() for part in normalized.split(',') if part.strip()]
  if len(parts) != 2:
    raise argparse.ArgumentTypeError('board size must look like 9x6 or 9,6.')
  return int(parts[0]), int(parts[1])


def print_result(result: CalibrationResult) -> None:
  print('\nCalibration complete')
  print(f'  image_size: {result.image_size[0]} x {result.image_size[1]}')
  print(f'  used_images: {len(result.used_images)}')
  print(f'  reprojection_error: {result.reprojection_error:.6f} px')
  print('  camera_matrix:')
  print(result.camera_matrix)
  print('  distortion_coefficients:')
  print(result.dist_coeffs.ravel())


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    description='Checkerboard camera calibration tool.')
  subparsers = parser.add_subparsers(dest='command', required=True)

  images = subparsers.add_parser('images', help='Calibrate from image files.')
  images.add_argument('--input', required=True, help='Image file or directory.')
  images.add_argument('--output', default='camera_calibration.yaml')
  images.add_argument('--preview-dir', default='')
  images.add_argument('--board-size', type=parse_board_size, default=(9, 6))
  images.add_argument('--board-cols', type=int, default=None)
  images.add_argument('--board-rows', type=int, default=None)
  images.add_argument('--square-size', type=float, default=1.0)
  images.add_argument('--min-images', type=int, default=10)

  video = subparsers.add_parser('video', help='Calibrate from video/camera frames.')
  video.add_argument('--source', default='1', help='Camera index or video path.')
  video.add_argument('--camera-type', choices=('webcam', 'csi', 'file'), default='webcam')
  video.add_argument('--sensor-id', type=int, default=0)
  video.add_argument('--output', default='camera_calibration.yaml')
  video.add_argument('--preview-dir', default='')
  video.add_argument('--board-size', type=parse_board_size, default=(9, 6))
  video.add_argument('--board-cols', type=int, default=None)
  video.add_argument('--board-rows', type=int, default=None)
  video.add_argument('--square-size', type=float, default=1.0)
  video.add_argument('--every-n', type=int, default=15)
  video.add_argument('--max-frames', type=int, default=80)
  video.add_argument('--width', type=int, default=1920)
  video.add_argument('--height', type=int, default=1080)
  video.add_argument('--capture-width', type=int, default=None)
  video.add_argument('--capture-height', type=int, default=None)
  video.add_argument('--fps', type=int, default=30)
  video.add_argument('--flip-method', type=int, default=0)
  video.add_argument('--min-images', type=int, default=10)

  capture = subparsers.add_parser('capture', help='Capture checkerboard images.')
  capture.add_argument('--source', default='1', help='Camera index or video path.')
  capture.add_argument('--camera-type', choices=('webcam', 'csi', 'file'), default='webcam')
  capture.add_argument('--sensor-id', type=int, default=0)
  capture.add_argument('--output-dir', required=True)
  capture.add_argument('--board-size', type=parse_board_size, default=(9, 6))
  capture.add_argument('--board-cols', type=int, default=None)
  capture.add_argument('--board-rows', type=int, default=None)
  capture.add_argument('--square-size', type=float, default=1.0)
  capture.add_argument('--width', type=int, default=1920)
  capture.add_argument('--height', type=int, default=1080)
  capture.add_argument('--capture-width', type=int, default=None)
  capture.add_argument('--capture-height', type=int, default=None)
  capture.add_argument('--fps', type=int, default=30)
  capture.add_argument('--flip-method', type=int, default=0)
  capture.add_argument('--count', type=int, default=20)

  return parser


def resolved_board_size(args: argparse.Namespace) -> BoardSize:
  if args.board_cols is not None or args.board_rows is not None:
    if args.board_cols is None or args.board_rows is None:
      raise ValueError('--board-cols and --board-rows must be used together.')
    return args.board_cols, args.board_rows
  return args.board_size


def main() -> None:
  parser = build_parser()
  args = parser.parse_args()
  board_size = resolved_board_size(args)

  if args.command == 'images':
    result = calibrate_from_images(
      args.input,
      board_size=board_size,
      square_size=args.square_size,
      output_path=args.output,
      preview_dir=args.preview_dir or None,
      min_images=args.min_images)
    print_result(result)
    print(f'  saved: {args.output}')
  elif args.command == 'video':
    result = calibrate_from_video(
      args.source,
      board_size=board_size,
      square_size=args.square_size,
      output_path=args.output,
      preview_dir=args.preview_dir or None,
      every_n=max(1, args.every_n),
      max_frames=max(1, args.max_frames),
      camera_width=args.width,
      camera_height=args.height,
      camera_fps=args.fps,
      camera_type=args.camera_type,
      sensor_id=args.sensor_id,
      capture_width=args.capture_width,
      capture_height=args.capture_height,
      flip_method=args.flip_method,
      min_images=args.min_images)
    print_result(result)
    print(f'  saved: {args.output}')
  elif args.command == 'capture':
    collect_checkerboard_images(
      args.source,
      Path(args.output_dir),
      board_size=board_size,
      square_size=args.square_size,
      camera_width=args.width,
      camera_height=args.height,
      camera_fps=args.fps,
      camera_type=args.camera_type,
      sensor_id=args.sensor_id,
      capture_width=args.capture_width,
      capture_height=args.capture_height,
      flip_method=args.flip_method,
      required_count=args.count)


if __name__ == '__main__':
  main()
