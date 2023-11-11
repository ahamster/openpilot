#!/usr/bin/env python3
from selfdrive.car import gen_empty_fingerprint

class TI:
  saved_candidate: str
  saved_finger = gen_empty_fingerprint()
  saved_CarInterface: object
  enabled = False