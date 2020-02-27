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
import bkmixture

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

SELECT_NUM = 9

'''
# グリッド番号(0.5m)
POINT_1_GRID = 487
POINT_2_GRID = 679
POINT_3_GRID = 684
POINT_4_GRID = 688
POINT_5_GRID = 496
POINT_6_GRID = 492
POINT_7_GRID = 256
POINT_8_GRID = 252
POINT_9_GRID = 247
'''

'''
# グリッド番号(1.0m)
POINT_1_GRID = 123
POINT_2_GRID = 171
POINT_3_GRID = 174
POINT_4_GRID = 176
POINT_5_GRID = 128
POINT_6_GRID = 126
POINT_7_GRID = 68
POINT_8_GRID = 65
POINT_9_GRID = 63
'''

'''
# グリッド番号(0.75m)
POINT_1_GRID = 228
POINT_2_GRID = 308
POINT_3_GRID = 296
POINT_4_GRID = 299
POINT_5_GRID = 235
POINT_6_GRID = 232
POINT_7_GRID = 123
POINT_8_GRID = 119
POINT_9_GRID = 116
'''


# グリッド番号(0.7m)
POINT_1_GRID = 259
POINT_2_GRID = 345
POINT_3_GRID = 348
POINT_4_GRID = 352
POINT_5_GRID = 266
POINT_6_GRID = 263
POINT_7_GRID = 130
POINT_8_GRID = 127
POINT_9_GRID = 123


# カルマンフィルタのパラメータ(2乗)
O_WALK = 0.01
O_RANGE = 300000
#O_RANGE = 300000
r = 1000

'''
# カルマンフィルタのパラメータ(ユークリッド距離)
O_WALK = 3000
O_RANGE = 554220
r = 1000
'''

# ベイジアンフィルタのパラメータ
x_now = np.mat([[TRUE_POSITION_1_X, TRUE_POSITION_1_Y]]).T
p_i = np.mat([[AP_A_X, AP_A_Y], [AP_B_X, AP_B_Y], [AP_C_X, AP_C_Y]]).T
#sigma_range = 23000000
sigma_range = 554220
now_grid = 0

# マップ情報のpath
map_path = '/Users/andy/Wifi_rtt/localize/map_07m_true.json'

# 読み込むデータのpath
read_path = '/Users/andy/Wifi_rtt/localize0117/'
read_file = 'position'

# データの出力先のpathとファイル名
write_path = '/Users/andy/Wifi_rtt/0114/newdata/bayesian_like/'
write_file = 'bay_loop'

# カルマンフィルタ(1)orベイジアンフィルタ(2)の選択orアプローチ(3)
filter = 2

def main():

  er_sum = 0
  num = 0

  #基地局からの距離
  dis_deviceA = 0
  dis_deviceB = 0
  dis_deviceC = 0

  c = np.mat([[1, 0], [0, 1]])
  p_1 = np.mat([[AP_A_X], [AP_A_Y]])
  p_2 = np.mat([[AP_B_X], [AP_B_Y]])
  p_3 = np.mat([[AP_C_X], [AP_C_Y]])

  draw = drawing.drawing()
  draw = setDraw(draw)

  '''
  # 歩いていると仮定した場合
  for i in range(POINT_NUM):
    truePosition = selectPosition(i + 1)
    draw.drawCircle(truePosition[0][0], truePosition[1][0], 100, "#050A11", True)'''

  
  if filter == 1:
      kf = kalman.KalmanFilter(TRUE_POSITION_1_X, TRUE_POSITION_1_Y, p_1, p_2, p_3)
      #kf = kalman.KalmanFilter(0, 0, p_1, p_2, p_3)
      kf.setModel(o_walk=O_WALK, o_range=O_RANGE)
      #kf.setModel(O_WALK, O_RANGE, r)
  elif filter == 2:
      bf = bayesian.BaysianFilter(POINT_1_GRID, p_i, sigma_range)
      bf.readMap(map_path)
  elif filter == 3:
      bkm = bkmixture.BKmixture(p_i)
      bkm.setKalman([TRUE_POSITION_1_X, TRUE_POSITION_1_Y], p_1, p_2, p_3, O_WALK, O_RANGE, r)
      bkm.setBayesian(sigma_range, map_path)

  with open(write_path + write_file + '8' + '.csv', mode='w') as fw:
     with open(read_path + read_file + 'A' + '.csv', encoding='utf-8', mode='r') as f:
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
            time = 0
            if filter == 1:
              kf.estimate()
              #kf.filter(Z)
              kf.filter(np.square(Z))
              x = kf.getStatus()
            elif filter == 2:
              grid = 1
              was_grid = 0
              for key in range(8):
                was_grid = grid
                bf.prediction()
                x, grid = bf.update(Z)
                if grid != was_grid:
                  time += 1
                print(str(grid) + ',' + str(was_grid))
            elif filter == 3:
              x, grid = bkm.estimate(Z)

            dis_deviceA = 0
            dis_deviceB = 0
            dis_deviceC = 0
            #er = error(x, num + 1)
            #er_sum += er
            fw.write(str(int(x[0, 0])) + ',' + str(int(x[1, 0])) + ',' + str(time) + ',' + '\n')
            mod = num % 8
            if mod == 0:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#EDAD0B", True)
            elif mod == 1:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#C7243A", True)
            elif mod == 2:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#D8E212", True)
            elif mod == 3:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#23AC0E", True)
            elif mod == 4:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#0086AB", True)
            elif mod == 5:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#3261AB", True)
            elif mod == 6:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#5D639E", True)
            elif mod == 7:
              draw.drawCircle(x[0, 0], x[1, 0], 100, "#A52175", True)
            num += 1
            #print('a')
            #draw.pause(0.1)
      draw.show()

  '''
  for num in range(POINT_NUM):
    filename = FILE_NAME
    truePosition = selectPosition(num + 1)
    print("now:" + str(now_grid))
    print("position:" + str(truePosition))
    draw.drawCircle(truePosition[0, 0], truePosition[1, 0], 100, "#050A11", True)
    if filter == 1:
      #kf = kalman.KalmanFilter(truePosition[0, 0], truePosition[1, 0], p_1, p_2, p_3)
      kf = kalman.KalmanFilter(0, 0, p_1, p_2, p_3)
      kf.setModel(O_WALK, O_RANGE, r)
    elif filter == 2:
      bf = bayesian.BaysianFilter(now_grid, p_i, sigma_range)
      # bf.readMap('/Users/andy/Wifi_rtt/mapInfo.json')
      bf.readMap(map_path)
    elif filter == 3:
      bkm = bkmixture.BKmixture(p_i)
      bkm.setKalman([truePosition[0, 0], truePosition[1, 0]], p_1, p_2, p_3, O_WALK, O_RANGE, r)
      bkm.setBayesian(sigma_range, map_path)

    with open(write_path + write_file + str(num+1) + '.csv', mode='w') as fw:
     with open(read_path + read_file + str(num+1) + '.csv', encoding='utf-8', mode='r') as f:
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
            if filter == 1:
              kf.estimate()
              #kf.filter(Z)
              kf.filter(np.square(Z))
              x = kf.getStatus()
            elif filter == 2:
              grid = 1
              was_grid = 0
              for key in range(5):
                was_grid = grid
                bf.prediction()
                x, grid = bf.update(Z)
                print(str(grid) + ',' + str(was_grid))
            elif filter == 3:
              x, grid = bkm.estimate(Z)
            
            dis_deviceA = 0
            dis_deviceB = 0
            dis_deviceC = 0
            er = error(x, num + 1)
            er_sum += er
            fw.write(str(x[0, 0]) + ',' + str(x[1, 0]) + ',' + str(er) + ',' + '\n')
            draw.drawCircle(x[0, 0], x[1, 0], 100, "#EDAD0B", True)
            #print('a')
            #draw.pause(0.1)
  '''
  #draw.show()
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

def setDraw(draw):
  x1 = 1330
  x2 = 3830
  x3 = 6330
  x4 = 8830
  y1 = 6340
  y2 = 10090
  y3 = 13090
  y4 = 2590
  width = 1800
  height = 600
  draw.setAxes(-1000, 13000, -1000, 18000)
  draw.drawSquare(x1, y1, width, height*2, "#23AC0E", False)
  draw.drawSquare(x1, y2, width, height*2, "#23AC0E", False)
  draw.drawSquare(x1, y3, width, height*2, "#23AC0E", False)
  draw.drawSquare(x2, y1, width, height*2, "#23AC0E", False)
  draw.drawSquare(x2, y2, width, height*2, "#23AC0E", False)
  draw.drawSquare(x2, y3, width, height*2, "#23AC0E", False)
  draw.drawSquare(x3, y1, width, height*2, "#23AC0E", False)
  draw.drawSquare(x3, y2, width, height*2, "#23AC0E", False)
  draw.drawSquare(x3, y3, width, height*2, "#23AC0E", False)
  draw.drawSquare(x3, y4, width, height, "#23AC0E", False)
  draw.drawSquare(x4, y1, width, height*2, "#23AC0E", False)
  draw.drawSquare(x4, y2, width, height*2, "#23AC0E", False)
  draw.drawSquare(x4, y3, width, height*2, "#23AC0E", False)
  draw.drawSquare(x4, y4, width, height, "#23AC0E", False)

  return draw




def calibration(getID, distanceMm):
  if getID == DEVICE_A_ID:
    distanceMm -= OFFSET_A
  elif getID == DEVICE_B_ID:
    distanceMm -= OFFSET_B
  elif getID == DEVICE_C_ID:
    distanceMm -= OFFSET_C

  distanceMm = (distanceMm ** 2 - 400 ** 2) ** 0.5

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
  global now_grid
  if num == 1:
    now_grid = POINT_1_GRID
    return np.mat([[TRUE_POSITION_1_X], [TRUE_POSITION_1_Y]])
  elif num == 2:
    now_grid = POINT_2_GRID
    return np.mat([[TRUE_POSITION_2_X], [TRUE_POSITION_2_Y]])
  elif num == 3:
    now_grid = POINT_3_GRID
    return np.mat([[TRUE_POSITION_3_X], [TRUE_POSITION_3_Y]])
  elif num == 4:
    now_grid = POINT_4_GRID
    return np.mat([[TRUE_POSITION_4_X], [TRUE_POSITION_4_Y]])
  elif num == 5:
    now_grid = POINT_5_GRID
    return np.mat([[TRUE_POSITION_5_X], [TRUE_POSITION_5_Y]])
  elif num == 6:
    now_grid = POINT_6_GRID
    return np.mat([[TRUE_POSITION_6_X], [TRUE_POSITION_6_Y]])
  elif num == 7:
    now_grid = POINT_7_GRID
    return np.mat([[TRUE_POSITION_7_X], [TRUE_POSITION_7_Y]])
  elif num == 8:
    now_grid = POINT_8_GRID
    return np.mat([[TRUE_POSITION_8_X], [TRUE_POSITION_8_Y]])
  elif num == 9:
    now_grid = POINT_9_GRID
    return np.mat([[TRUE_POSITION_9_X], [TRUE_POSITION_9_Y]])

if __name__ == '__main__':
  main()
