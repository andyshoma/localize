#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches as pat
from matplotlib import image as mpimg
import math
import numpy as np
import drawing
import kalman
import bayesian

#offset
'''
OFFSET_A = 3941
OFFSET_B = 4383
OFFSET_C = 4804

OFFSET_A = 1619
OFFSET_B = 3987
OFFSET_C = 1620
'''
OFFSET_A = 3481
OFFSET_B = 4860
OFFSET_C = 3109

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

#ground truth
TRUE_POSITION_1_X = 3330
TRUE_POSITION_1_Y = 8940

TRUE_POSITION_2_X = 3330
TRUE_POSITION_2_Y = 11940

TRUE_POSITION_3_X = 5830
TRUE_POSITION_3_Y = 11940

TRUE_POSITION_4_X = 8330
TRUE_POSITION_4_Y = 11940

TRUE_POSITION_5_X = 8330
TRUE_POSITION_5_Y = 8940

TRUE_POSITION_6_X = 5830
TRUE_POSITION_6_Y = 8940

TRUE_POSITION_7_X = 8330
TRUE_POSITION_7_Y = 4440

TRUE_POSITION_8_X = 5830
TRUE_POSITION_8_Y = 4440

TRUE_POSITION_9_X = 3330
TRUE_POSITION_9_Y = 4440

#ファイル名
FILE_NAME = 'bayesian'

#計測点の数
POINT_NUM = 9

SELECT_NUM = 4

# カルマンフィルタのパラメータ
O_WALK = 0.01
O_RANGE = 300000

# ベイジアンフィルタのパラメータ
x_now = np.mat([[TRUE_POSITION_1_X, TRUE_POSITION_1_Y]]).T
c = 3000
p_i = np.mat([[AP_A_X, AP_A_Y], [AP_B_X, AP_B_Y], [AP_C_X, AP_C_Y]]).T
sigma_range = 30000000

def main():
  #基地局からの距離
  dis_deviceA = 0
  dis_deviceB = 0
  dis_deviceC = 0

  '''c = np.mat([[1, 0], [0, 1]])
  p_1 = np.mat([[AP_A_X], [AP_A_Y]])
  p_2 = np.mat([[AP_B_X], [AP_B_Y]])
  p_3 = np.mat([[AP_C_X], [AP_C_Y]])'''

  draw = drawing.drawing()
  draw.setAxes(-1000, 13000, -1000, 18000)
  draw.drawSquare(0, 0, 11800, 16890, "#23AC0E", False)
  
  # 一点で観測した場合
  truePosition = selectPosition(SELECT_NUM)
  draw.drawCircle(truePosition[0, 0], truePosition[1, 0], 100, "#050A11", True)

  '''
  # 歩いていると仮定した場合
  for i in range(POINT_NUM):
    truePosition = selectPosition(i + 1)
    draw.drawCircle(truePosition[0][0], truePosition[1][0], 100, "#050A11", True)'''

  for num in range(POINT_NUM):
    filename = FILE_NAME
    truePosition = selectPosition(1)
    #kf = kalman.KalmanFilter(0, 0, p_1, p_2, p_3)
    #kf.setModel(O_WALK, O_RANGE)
    bf = bayesian.BaysianFilter(x_now, c, p_i, sigma_range)
    with open('/Users/andy/Wifi_rtt/bayesian/bays' + str(num+1) + '.csv', mode='w') as fw:
     with open('/Users/andy/Wifi_rtt/position' + str(num+1) + '.csv', encoding='utf-8', mode='r') as f:
      f.readline()
      for line in f:
          data = line.split(',')
          distanceMm = int(data[0])
          getID = int(data[4])
          #truenum = int(data[5])

          #ファイルから取得したデータをcalibrate
          if getID == DEVICE_A_ID:
            dis_deviceA = calibration(getID, distanceMm)
          elif getID == DEVICE_B_ID:
            dis_deviceB = calibration(getID, distanceMm)
          elif getID == DEVICE_C_ID:
            dis_deviceC = calibration(getID, distanceMm)
            
          #測距結果を用いて測位
          if dis_deviceA != 0 and dis_deviceB != 0 and dis_deviceC != 0:
            Z = np.mat([[dis_deviceA], [dis_deviceB], [dis_deviceC]])
            #kf.estimate()
            #kf.filter(Z)
            bf.prediction()
            x = bf.update(Z)
            print(x)
            dis_deviceA = 0
            dis_deviceB = 0
            dis_deviceC = 0
            #x = kf.getStatus()
            er = error(x, num+1)
            fw.write(str(x[0, 0]) + ',' + str(x[1, 0]) + ',' + str(er) + ',' + '\n')
            draw.drawCircle(x[0,0], x[1,0], 100, "#EDAD0B", True)
            #draw.pause(0.3)

  draw.show()
  
  return

'''
def main():

  #基地局からの距離
  dis_deviceA = 0
  dis_deviceB = 0
  dis_deviceC = 0

  est_position_weight_x = 0
  est_position_weight_y = 0
  est_position_tri_x = 0
  est_position_tri_y = 0

  for num in range(POINT_NUM):
    filename = FILE_NAME + str(num+1)
    with open('/Users/andy/Wifi_rtt/' + filename + '.csv', mode='w') as fw:
      with open('/Users/andy/Wifi_rtt/wifirtt_0807_1/ranging_triple_' + str(num + 1) + '.csv', encoding='utf-8', mode='r') as f:
      #with open('/Users/andy/Wifi_rtt/position' + str(num + 1) + '.csv', encoding='utf-8', mode='r') as f:
        f.readline()
        for line in f:
          data = line.split(',')
          distanceMm = int(data[1])
          getID = int(data[0])

          #ファイルから取得したデータをcalibrate
          if getID == DEVICE_A_ID:
            dis_deviceA = calibration(getID, distanceMm)
          elif getID == DEVICE_B_ID:
            dis_deviceB = calibration(getID, distanceMm)
          elif getID == DEVICE_C_ID:
            dis_deviceC = calibration(getID, distanceMm)

          #測距結果を用いて測位
          if dis_deviceA != 0 and dis_deviceB != 0 and dis_deviceC != 0:
            est_position_weight_x, est_position_weight_y = weighted(dis_deviceA, dis_deviceB, dis_deviceC)
            est_position_tri_x, est_position_tri_y = triangulation(dis_deviceA, dis_deviceB, dis_deviceC)
            truePositionX, truePositionY = selectPosition(num+1)
            #accurate = math.sqrt((est_position_x-truePositionX) * (est_position_x-truePositionX)+(est_position_y-truePositionY) * (est_position_y-truePositionY))
            #fw.write(str(est_position_x) + ',' + str(est_position_y) + ',' + str(accurate) + '\n')

    draw = drawing()
    draw.drawCircle(AP_A_X, AP_A_Y, dis_deviceA, "#C7243A", False)
    draw.drawCircle(AP_B_X, AP_B_Y, dis_deviceB, "#007AB7", False)
    draw.drawCircle(AP_C_X, AP_C_Y, dis_deviceC, "#009250", False)
    draw.drawCircle(est_position_weight_x,
                    est_position_weight_y, 100, "#EDAD0B", True)
    draw.drawCircle(est_position_tri_x, est_position_tri_y,
                    100, "#A52175", True)
    draw.drawCircle(truePositionX, truePositionY, 200, "#050A11", True)
    draw.drawView()

  return
'''

def calibration(getID, distanceMm):
  if getID == DEVICE_A_ID:
    distanceMm -= OFFSET_A
  elif getID == DEVICE_B_ID:
    distanceMm -= OFFSET_B
  elif getID == DEVICE_C_ID:
    distanceMm -= OFFSET_C

  return distanceMm

def error(x, num):
  position = selectPosition(num)
  er = int(((position[0,0] - x[0,0]) ** 2 + (position[1,0] - x[1,0]) ** 2) ** 0.5)

  return er


def triangulation(disA, disB, disC):
  va_1 = ((disB * disB - disA * disA) - (AP_B_X * AP_B_X - AP_A_X *
                                         AP_A_X) - (AP_B_Y * AP_B_Y - AP_A_Y * AP_A_Y)) / 2
  vb_1 = ((disB * disB - disC * disC) - (AP_B_X * AP_B_X - AP_C_X *
                                         AP_C_X) - (AP_B_Y * AP_B_Y - AP_C_Y * AP_C_Y)) / 2

  x1 = ((AP_C_Y - AP_B_Y) * va_1 - (AP_A_Y - AP_B_Y) * vb_1) / ((AP_A_X -
                                                                 AP_B_X) * (AP_C_Y - AP_B_Y) - (AP_C_X - AP_B_X) * (AP_A_Y - AP_B_Y))
  y1 = (va_1 - x1 * (AP_A_X - AP_B_X)) / (AP_A_Y - AP_B_Y)

  '''va_2 = ((disA * disA - disB * disB) - (AP_A_X * AP_A_X - AP_B_X * AP_B_X) - (AP_A_Y * AP_A_Y - AP_B_Y * AP_B_Y)) / 2
  vb_2 = ((disA * disA - disC * disC) - (AP_A_X * AP_A_X - AP_C_X * AP_C_X) - (AP_A_Y * AP_A_Y - AP_C_Y * AP_C_Y)) / 2

  x2 = ((AP_C_Y - AP_A_Y) * va_2 - (AP_B_Y - AP_A_Y) * vb_2) / ((AP_B_X - AP_A_X) * (AP_C_Y - AP_A_Y) - (AP_C_X - AP_A_X) * (AP_B_Y - AP_A_Y))
  y2 = (va_2 - x2 * (AP_B_X - AP_A_X)) / (AP_B_Y - AP_A_Y)

  va_3 = ((disC * disC - disA * disA) - (AP_C_X * AP_C_X - AP_A_X * AP_A_X) - (AP_C_Y * AP_C_Y - AP_A_Y * AP_A_Y)) / 2
  vb_3 = ((disC * disC - disB * disB) - (AP_C_X * AP_C_X - AP_B_X * AP_B_X) - (AP_C_Y * AP_C_Y - AP_B_Y * AP_B_Y)) / 2

  x3 = ((AP_A_Y - AP_C_Y) * va_3 - (AP_B_Y - AP_C_Y) * vb_3) / ((AP_B_X - AP_C_X) * (AP_A_Y - AP_C_Y) - (AP_A_X - AP_C_X) * (AP_B_Y - AP_C_Y))
  y3 = (va_3 - x3 * (AP_A_X - AP_C_X)) / (AP_A_Y - AP_C_Y)'''

  return x1, y1


def weighted(disA, disB, disC):
  weightA = 1 / disA
  weightB = 1 / disB
  weightC = 1 / disC
  x0 = (AP_A_X * weightA + AP_B_X * weightB + AP_C_X *
        weightC) / (weightA + weightB + weightC)
  y0 = (AP_A_Y * weightA + AP_B_Y * weightB + AP_C_Y *
        weightC) / (weightA + weightB + weightC)
  #print("x0: " + str(x0) + ", y0: " + str(y0) + "\n")
  return x0, y0

def selectPosition(num):
  if num == 1:
    return np.mat([[TRUE_POSITION_1_X], [TRUE_POSITION_1_Y]])
  elif num == 2:
    return np.mat([[TRUE_POSITION_2_X], [TRUE_POSITION_2_Y]])
  elif num == 3:
    return np.mat([[TRUE_POSITION_3_X], [TRUE_POSITION_3_Y]])
  elif num == 4:
    return np.mat([[TRUE_POSITION_4_X], [TRUE_POSITION_4_Y]])
  elif num == 5:
    return np.mat([[TRUE_POSITION_5_X], [TRUE_POSITION_5_Y]])
  elif num == 6:
    return np.mat([[TRUE_POSITION_6_X], [TRUE_POSITION_6_Y]])
  elif num == 7:
    return np.mat([[TRUE_POSITION_7_X], [TRUE_POSITION_7_Y]])
  elif num == 8:
    return np.mat([[TRUE_POSITION_8_X], [TRUE_POSITION_8_Y]])
  elif num == 9:
    return np.mat([[TRUE_POSITION_9_X], [TRUE_POSITION_9_Y]])

if __name__ == '__main__':
  main()
