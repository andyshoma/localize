#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches as pat
from matplotlib import image as mpimg
import math
import numpy as np
import sys

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

# ファイル名
FILE_NAME = '/Users/andy/Wifi_rtt/position'

# ファイルの数
FILE_NUM = 9

def main():
    args = sys.argv
    with open('/Users/andy/Wifi_rtt/offset/offset.csv', mode='w') as fw:
      str1 = "200372,200359,200358\n"
      fw.write(str1)
      for num in range(FILE_NUM):
        filename = FILE_NAME + str(num+1) + '.csv'
        average = getAverage(filename, num+1)
        str1 = str(average[0, 0]) + ',' + str(average[1, 0]) + ',' + str(average[2, 0]) + '\n'
        fw.write(str1)
    
    return

'''
基地局と測位点との距離
・引数
    ・position：測位点の番号
・返り値
    ・基地局からの距離の真値（３次元行列）
'''
def getDistance(position):
    AP_A = None
    AP_B = None
    AP_C = None
    distanceA = None
    distanceB = None
    distanceC = None

    AP_A = np.mat([[AP_A_X], [AP_A_Y]])
    AP_B = np.mat([[AP_B_X], [AP_B_Y]])
    AP_C = np.mat([[AP_C_X], [AP_C_Y]])

    distanceA = ((position[0, 0] - AP_A[0, 0]) ** 2 + (position[1, 0] - AP_A[1, 0]) ** 2) ** 0.5
    distanceB = ((position[0, 0] - AP_B[0, 0]) ** 2 + (position[1, 0] - AP_B[1, 0]) ** 2) ** 0.5
    distanceC = ((position[0, 0] - AP_C[0, 0]) ** 2 + (position[1, 0] - AP_C[1, 0]) ** 2) ** 0.5

    return np.mat([[distanceA], [distanceB], [distanceC]])
    
'''
端末IDごとの平均値を取得するための関数
・引数
    ・filename：読み込むファイルの名前
    ・num：positionの番号
・返り値
    ・ave：平均値（３次元行列）
'''
def getAverage(filename, num):
    sum = np.mat([[0], [0], [0]])
    length = 0
    ave = None
    dev = None
    potision = None
    distance = np.mat([[0], [0], [0]])
    timestamp = 0

    with open(filename, encoding='utf-8', mode='r') as f:
        position = selectPosition(num)
        distance = getDistance(position)
        for line in f:
            data = line.split(',')
            distanceMm = int(data[0])
            ID = int(data[4])
            if timestamp == 0:
              timestamp = int(data[3])
              distance = distanceID(ID, distanceMm, distance)
              distanceNum = 1
            elif int(data[3]) < timestamp + 10 and int(data[3]) > timestamp - 10:
              distance = distanceID(ID, distanceMm, distance)
              distanceNum += 1
            else:
              distance = np.mat([[0], [0], [0]])
              timestamp = int(data[3])
              distance = distanceID(ID, distanceMm, distance)
              distanceNum = 1
            
            if distanceNum == 3:
              sum = getSum(distance, getDistance(position), sum)
              length += 1
              timestamp = 0

        ave = sum / length
        print(str(ave))

    return ave

def getDev(filename, num):
    with open(filename, encoding='utf-8', mode='r') as f:
        f.readline()
        for line in f:
            data = line.split(',')
            distance = int(data[0])

'''
端末IDごとに測距誤差の合計を計算する関数
・引数
    ・distanceMm：測距結果
    ・trueDistance：測距の真値
    ・ID：基地局にしている端末のID
    ・sum：現在までの合計値（３次元行列）
・返り値
    ・sum：計算後の合計値（３次元行列）
'''
def getSum(distance, trueDistance, sum):
    sum[0][0] = sum[0, 0] + (distance[0, 0] - trueDistance[0, 0])
    sum[1][0] = sum[1, 0] + (distance[1, 0] - trueDistance[1, 0])
    sum[2][0] = sum[2, 0] + (distance[2, 0] - trueDistance[2, 0])
    return sum

'''
測位点を選択する関数
・引数
    ・num：positionの番号
・返り値
    ・測位点の座標(x, y)
'''
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


def distanceID(getID, distanceMm, distance):
  if getID == DEVICE_A_ID:
    distance[0, 0] = distanceMm
  elif getID == DEVICE_B_ID:
    distance[1, 0] = distanceMm
  elif getID == DEVICE_C_ID:
    distance[2, 0] = distanceMm
    
  return distance
if __name__ == '__main__':
  main()
