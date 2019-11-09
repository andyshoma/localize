#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches as pat
from matplotlib import image as mpimg
import math

#offset
OFFSET_A = 3941
OFFSET_B = 4383
OFFSET_C = 4804

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
FILE_NAME = 'localization'

#計測点の数
POINT_NUM = 9

NUM = 1

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


def calibration(getID, distanceMm):
  if getID == DEVICE_A_ID:
    distanceMm -= OFFSET_A
  elif getID == DEVICE_B_ID:
    distanceMm -= OFFSET_B
  elif getID == DEVICE_C_ID:
    distanceMm -= OFFSET_C

  return distanceMm


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

  #print("x1: " + str(x1) + ", y1: " + str(y1))
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
    return TRUE_POSITION_1_X, TRUE_POSITION_1_Y
  elif num == 2:
    return TRUE_POSITION_2_X, TRUE_POSITION_2_Y
  elif num == 3:
    return TRUE_POSITION_3_X, TRUE_POSITION_3_Y
  elif num == 4:
    return TRUE_POSITION_4_X, TRUE_POSITION_4_Y
  elif num == 5:
    return TRUE_POSITION_5_X, TRUE_POSITION_5_Y
  elif num == 6:
    return TRUE_POSITION_6_X, TRUE_POSITION_6_Y
  elif num == 7:
    return TRUE_POSITION_7_X, TRUE_POSITION_7_Y
  elif num == 8:
    return TRUE_POSITION_8_X, TRUE_POSITION_8_Y
  elif num == 9:
    return TRUE_POSITION_9_X, TRUE_POSITION_9_Y


class drawing():

  '''
  コンストラクタ
  '''

  def __init__(self):
    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(111)
    plt.axes().set_aspect('equal')

  '''
  円を書くためのメソッド
  引数は以下の５つ
    ・x, y：long型　円の中心座標
    ・radius：long型　円の半径
    ・color：String型　縁の色（例：red, blue, #FFFFFF, #C7243A）
    ・fill：bool型　図形の塗りつぶし（True：塗りつぶし, False：塗りつぶしなし）
  '''

  def drawCircle(self, x, y, radius, color, fill):
    circle = pat.Circle(xy=(x, y), radius=radius, color=color, fill=fill)
    self.ax.add_patch(circle)

  '''
  座標軸の設定
  引数は以下の４つ
  　・x_min, x_max：x軸の最小値と最大値（x_min < x < x_max)
  　・y_min, y_max：y軸の最小値と最大値（y_min < y < y_max）
  '''

  def setAxes(self, x_min, x_max, y_min, y_max):
    self.ax.set_xlim(x_min, x_max)
    self.ax.set_ylim(y_min, y_max)
    self.ax.invert_xaxis()

  '''
  四角形を書くためのメソッド
  引数は以下の6つ
    ・x, y：long型　四角形の始点の座標
    ・width：long型　横の長さ
    ・height：long型　縦の長さ
    ・color：String型　縁の色（例：red, blue, #FFFFFF, #C7243A）
    ・fill：bool型　図形の塗りつぶし（True：塗りつぶし, False：塗りつぶしなし）
  '''

  def drawSquare(self, x, y, width, height, color, fill):
    rec = pat.Rectangle(xy=(x, y), width=width, height=height,
                        color=color, fill=fill)
    self.ax.add_patch(rec)

  '''
  背景画像の設定
  引数は以下の１つ
    ・img_pass：String型　画像の相対パス
  '''

  def imshow(self, img_pass):
    img = mpimg.imread(img_pass)
    self.ax.imshow(img)

  '''
  図形の連続描写
  引数は以下の1つ
  　・interval：long型　図形の表示間隔（秒単位）
  '''

  def pause(self, interval):
    plt.pause(interval)

  
  '''
  図形の描写
  引数はなし
  '''
  def drawView(self):
    plt.show()

if __name__ == '__main__':
  main()
