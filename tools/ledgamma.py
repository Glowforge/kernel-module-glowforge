#!/usr/bin/env python

import sys
import math

MAX_BRIGHTNESS = 1023
NUM_STEPS = 256
FRAC_BITS = 8

def gentable(gamma):
  for i in xrange(0,NUM_STEPS):
    value = MAX_BRIGHTNESS * (float(i)/(NUM_STEPS-1)) ** gamma;
    sys.stdout.write('%d, ' % math.floor(value*(1<<FRAC_BITS)))
  sys.stdout.write('\n')

if __name__ == '__main__':
  gentable(float(sys.argv[1]))
