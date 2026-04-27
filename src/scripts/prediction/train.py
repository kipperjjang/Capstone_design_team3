#!/usr/bin/env python3
try:
  from .gru.train import main
except ImportError:
  from gru.train import main


if __name__ == '__main__':
  main()

