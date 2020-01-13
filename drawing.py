#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches as pat
from matplotlib import image as mpimg


class drawing():

  def __init__(self):
    '''[コンストラクタ]
    '''
    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(111)
    plt.axes().set_aspect('equal')


  def setAxes(self, x_min, x_max, y_min, y_max):
    '''[座標軸の設定]
    
    Args:
        x_min ([long]): [x軸の最小値]
        x_max ([long]): [x軸の最大値]
        y_min ([long]): [y軸の最小値]
        y_max ([long]): [y軸の最大値]
    '''
    self.ax.set_xlim(x_min, x_max)
    self.ax.set_ylim(y_min, y_max)


  def drawCircle(self, x, y, radius, color, fill):
    '''[円を書くメソッド]
    
    Args:
        x ([long]): [円の中心のx座標]
        y ([long]): [円の中心のy座標]
        radius ([long]): [円の半径]
        color ([String]): [円の色]
        fill ([bool]): [図形の塗りつぶし（True：塗りつぶし, False：塗りつぶしなし）]
    '''
    circle = pat.Circle(xy=(x, y), radius=radius, color=color, fill=fill)
    self.ax.add_patch(circle)


  def drawSquare(self, x, y, width, height, color, fill):
    '''[四角形を書くメソッド]
    
    Args:
        x ([long]): [四角形の始点のx座標]
        y ([type]): [四角形の始点のy座標]
        width ([long]): [横の長さ]
        height ([long]): [縦の長さ]
        color ([String]): [四角形の色]
        fill ([bool]): [図形の塗り潰し（True：塗りつぶし, False：塗りつぶしなし）]
    '''
    rec = pat.Rectangle(xy=(x, y), width=width, height=height,
                        color=color, fill=fill)
    self.ax.add_patch(rec)


  def imshow(self, img_pass):
    '''[景画像の設定]
    
    Args:
        img_pass ([String]): [画像の相対パス]
    '''
    img = mpimg.imread(img_pass)
    self.ax.imshow(img)


  def pause(self, interval):
    '''[図形の連続描画]
    
    Args:
        interval ([long]): [図形の表示間隔（秒単位）]
    '''
    plt.pause(interval)


  def show(self):
    '''[図形の描画]
    '''
    plt.show()
