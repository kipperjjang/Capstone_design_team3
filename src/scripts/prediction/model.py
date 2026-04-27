#!/usr/bin/env python3
try:
  from .gru.model import FutureGRUPredictor
except ImportError:
  from gru.model import FutureGRUPredictor

__all__ = ['FutureGRUPredictor']

