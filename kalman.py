import math
import numpy as np


class KalmanFilter:
  # 状態空間モデル
  X = None
  A = None

  # state x = A * x_ + B * u + w, w~N(0,Q)
  u = None
  Q = None
  R = None

  # ヤコビ行列
  C = None

  # 事前状態推定値
  x_ = None

  # 事前誤差共分散行列
  P_ = None

  # 状態推定値
  x = None

  # 事後誤差共分散行列
  P = None

  # カルマンゲイン行列
  G = None

  # 基地局の座標
  P_1 = None
  P_2 = None
  P_3 = None

  '''
  コンストラクタ
  引数は以下の２つ
  initial_x, initial_y : long型のリスト　初期位置
  '''

  def __init__(self, initial_x, initial_y, p_1, p_2, p_3):
    self.x = np.mat([[initial_x], [initial_y]])
    self.P_1 = p_1
    self.P_2 = p_2
    self.P_3 = p_3

  '''
  状態モデルと観測モデルの設定
  引数は５つ
  状態モデル x = x + v, v~N(0,Q)
  観測モデル z_ = h(x) + w, w~N(0,R)
  '''

  def setModel(self, o_walk, o_range):
    #self.C = c
    self.A = np.mat([[1, 0], [0, 1]])
    self.Q = np.mat([[o_walk, 0], [0, o_walk]])
    self.R = np.mat([[o_range, 0, 0], [0, o_range, 0], [0, 0, o_range]])
    self.P = np.mat([[0, 0], [0, 0]])

  # 推定
  '''
  事前状態推定値　x_(k) = A * x(k-1)
  線型近似　ヤコビアン計算
  事前誤差共分散行列　P_(k) = A * P(k-1) * A.T + Q(k-1)
  '''

  def estimate(self):
    self.x_ = self.A * self.x
    self.C = self.C_(self.x_)
    self.P_ = self.P + self.Q
    print("x_" + str(self.x))
    print("P_ : " + str(self.P_))
    print("C : " + str(self.C))

  # 更新
  '''
  カルマンゲイン　G(k) = P_(k) * C(k) * (c(k) * P_(k) * c(k).T + R(k)).I
  状態推定値　x(k) = x_(k) + G(k) * (z(k) - h(x_(k)))
  事後誤差共分散行列　P(k) = (I - g(k) * c(k)) * P_(k)
  '''

  def filter(self, Z):
    S = self.C * self.P_ * self.C.T + self.R
    self.G = self.P_ * self.C.T * S.I
    print('G: ' + str(self.G))
    I = np.mat([[1, 0], [0, 1]])
    self.P = (I - self.G * self.C) * self.P_
    print('P: ' + str(self.P))
    self.x = self.x_ + self.G * (Z - self.h(self.x_))
    print("x" + str(self.x))

  # h(x_k)
  def h(self, x):
    return np.mat([[self.h_(x, self.P_1)], [self.h_(x, self.P_2)], [self.h_(x, self.P_3)]])

  # ２乗の場合

  def h_(self, x, p):
    return (x[0, 0] - p[0, 0]) ** 2 + (x[1, 0] - p[1, 0]) ** 2

  # ヤコビアン
  def C_(self, x_):
    return np.mat([[2 * (x_[0, 0] - self.P_1[0, 0]), 2 * (x_[1, 0] - self.P_1[1, 0])], [2 * (x_[0, 0] - self.P_2[0, 0]), 2 * (x_[1, 0] - self.P_2[1, 0])], [2 * (x_[0, 0] - self.P_3[0, 0]), 2 * (x_[1, 0] - self.P_3[1, 0])]])

  '''
  # ユークリッド距離の場合
  def h_(self, x, p):
    return math.sqrt((x[0, 0] - p[0, 0]) ** 2 + (x[1, 0] - p[1, 0]) ** 2)

  # ヤコビアン
  def C_(self, x_):
    h_1 = self.h_(self.x_, self.P_1)
    h_2 = self.h_(self.x_, self.P_2)
    h_3 = self.h_(self.x_, self.P_3)
    return np.mat([[((self.x_[0, 0]-self.P_1[0, 0])/h_1), ((self.x_[1, 0]-self.P_1[1, 0])/h_1)], [((self.x_[0, 0]-self.P_2[0, 0])/h_2), ((self.x_[1, 0]-self.P_1[1, 0])/h_2)], [((self.x_[0, 0]-self.P_1[0, 0])/h_3), ((self.x_[1, 0]-self.P_1[1, 0])/h_3)]])
  '''

  # 状態情報を取得
  def getStatus(self):
    return self.x