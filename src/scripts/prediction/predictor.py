#!/usr/bin/env python3
try:
  from .gru.predictor import TargetPredictor
except ImportError:
  from gru.predictor import TargetPredictor

__all__ = ['TargetPredictor']

