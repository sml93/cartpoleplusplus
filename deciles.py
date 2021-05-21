#!/usr/bin/env python2
import sys
import numpy as np
print(np.percentile(list(map(float, sys.stdin.readlines())),
                    np.linspace(0, 100, 11)))
