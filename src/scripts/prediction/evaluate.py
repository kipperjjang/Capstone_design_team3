#!/usr/bin/env python3
try:
  from .gru.evaluate import main
except ImportError:
  from gru.evaluate import main


if __name__ == '__main__':
  main()

