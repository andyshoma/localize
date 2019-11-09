#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches as pat
from matplotlib import image as mpimg


class drawing():

  '''
  コンストラクタ
  '''

  def __init__(self):
    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(111)
    plt.axes().set_aspect('equal')

  '''
  座標軸の設定
  引数は以下の４つ
  　・x_min, x_max：x軸の最小値と最大値（x_min < x < x_max)
  　・y_min, y_max：y軸の最小値と最大値（y_min < y < y_max）
  '''

  def setAxes(self, x_min, x_max, y_min, y_max):
    self.ax.set_xlim(x_min, x_max)
    self.ax.set_ylim(y_min, y_max)

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

  def show(self):
    plt.show()
