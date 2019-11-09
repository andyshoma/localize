#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import bayesian
import numpy as np
import random

#accesspointAの座標
AP_A_X = 9330
AP_A_Y = 12940
DEVICE_A_ID = 200372

#accesspointBの座標
AP_B_X = 9330
AP_B_Y = 2940
DEVICE_B_ID = 200359

#accesspointCの座標
AP_C_X = 1830
AP_C_Y = 6940
DEVICE_C_ID = 200358

def main():
    x = np.mat([0, 0]).T
    c = 30
    p_i = np.mat([[AP_A_X, AP_A_Y], [AP_B_X, AP_B_Y], [AP_C_X, AP_C_Y]]).T
    print(p_i)
    sigma_range = 300000
    y_n = np.mat([[1000], [2000], [1500]])

    now = bayesian.BaysianFilter(x, c, p_i, sigma_range)
    now.prediction()
    now.update(y_n)

if __name__ == '__main__':
  main()
