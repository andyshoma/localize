#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import bayesian
import numpy as np
import random
import mapMaker
import bkmixture
import search
import json
import drawing

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

path = "/Users/andy/Wifi_rtt/localize/"
filename = "sample.json"

# 読み込むデータのpath
read_path = '/Users/andy/Wifi_rtt/localize0117/'
read_file = 'position'

def main():
    
    #search1(path+filename)
    makeGridMap(grid_size=700, obstacle=True)
    printMap()
    

def makeObstacleMap():
  x1 = [1330, 3130]
  x2 = [3830, 5630]
  x3 = [6330, 8130]
  x4 = [8830, 10630]
  y1 = [6340, 7540]
  y2 = [10090, 11290]
  y3 = [13090, 14290]
  y4 = [2590, 3190]
  

def makeGridMap(grid_size, obstacle):
  map = mapMaker.MapMaker()
  x1 = [1330, 3130]
  x2 = [3830, 5630]
  x3 = [6330, 8130]
  x4 = [8830, 10630]
  y1 = [6340, 7540]
  y2 = [10090, 11290]
  y3 = [13090, 14290]
  y4 = [2590, 3190]

  if obstacle == True:
    map.addObstacle(x1, y1)
    map.addObstacle(x1, y2)
    map.addObstacle(x1, y3)
    map.addObstacle(x2, y1)
    map.addObstacle(x2, y2)
    map.addObstacle(x2, y3)
    map.addObstacle(x3, y1)
    map.addObstacle(x3, y2)
    map.addObstacle(x3, y3)
    map.addObstacle(x3, y4)
    map.addObstacle(x4, y1)
    map.addObstacle(x4, y2)
    map.addObstacle(x4, y3)
    map.addObstacle(x4, y4)
    #map.printObstacle()

  map.setMapSize([0, 11800], [0, 16890])
  map.setGridSize(grid_size)
  map.gridDivision()
  map.makeJson2()
  map.removeAdjList()
  map.saveFile(path, filename)

def search1(path):
  sea = search.Search()
  sea.readJsonFile(path)
  print(sea.searching([TRUE_POSITION_9_X, TRUE_POSITION_9_Y]))

def printMap():
  draw = drawing.drawing()
  draw.setAxes(x_min=-1000, x_max=13000, y_min=-1000, y_max=18000)
  draw = drawObstacle(draw)
  map_data = readJsonFile(path + filename)
  for key in map_data.keys():
    is_enable = map_data[key]['enable']
    x_cood = map_data[key]['cood'][0]
    y_cood = map_data[key]['cood'][1]
    if is_enable == True:
      draw.drawCircle(x_cood, y_cood, 50, "#A4C520", True)
    if is_enable == False:
      draw.drawCircle(x_cood, y_cood, 50, "#717171", True)

  draw.show()


def drawObstacle(draw):
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


def readJsonFile(path):
  with open(path, mode='r') as fr:
    json_data = json.load(fr)

  return json_data

'''
def makeCSV(path):
  data = []

  for num in range(9):
    with open(read_path + read_file + str(num + 1) + '.csv', encoding='utf-8', mode='r') as fr:
      list = [s.strip() for s in fr.readlines()]
      data.append(list)

    count = 0
    with open(path, mode='w') as fw:
      while count < 50:
        

def write3():
  first = None
  second = None
  third = None
'''
  


if __name__ == '__main__':
  main()
