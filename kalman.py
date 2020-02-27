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


  def __init__(self, initial_x, initial_y, p_1, p_2, p_3):
    '''[コンストラクタ]
    
    Args:
        initial_x ([long]): [初期位置のx座標]
        initial_y ([long]): [初期位置のy座標]
        p_1 ([mat]): [基地局1の座標]
        p_2 ([mat]): [基地局2の座標]
        p_3 ([mat]): [基地局3の座標]
    '''
    self.x = np.mat([[initial_x], [initial_y]])
    self.P_1 = p_1
    self.P_2 = p_2
    self.P_3 = p_3


  def setModel(self, o_walk, o_range, r=0):
    '''[状態モデルと観測モデルの設定]

    状態モデル x = x + v, v~N(0,Q)
    観測モデル z_ = h(x) + w, w~N(0,R)
    
    Args:
        o_walk ([int]): [状態モデルの誤差分散]
        o_range ([int]): [観測モデルの誤差分散]
    '''
    #self.C = c
    self.A = np.mat([[1, 0], [0, 1]])
    self.Q = np.mat([[o_walk, 0], [0, o_walk]])
    self.R = np.mat([[o_range, 0, 0], [0, o_range, 0], [0, 0, o_range]])
    self.P = np.mat([[r, 0], [0, r]])


  def estimate(self):
    '''[推定ステップ]

    事前状態推定値　x_(k) = A * x(k-1)
    線型近似　ヤコビアン計算
    事前誤差共分散行列　P_(k) = A * P(k-1) * A.T + Q(k-1)

    '''
    self.x_ = self.A * self.x
    self.C = self.C_(self.x_)
    self.P_ = self.P + self.Q


  def filter(self, Z):
    '''[更新ステップ]

    カルマンゲイン　G(k) = P_(k) * C(k) * (c(k) * P_(k) * c(k).T + R(k)).I
    状態推定値　x(k) = x_(k) + G(k) * (z(k) - h(x_(k)))
    事後誤差共分散行列　P(k) = (I - g(k) * c(k)) * P_(k)
    
    Args:
        Z ([mat]): [観測値]
    '''
    S = self.C * self.P_ * self.C.T + self.R
    self.G = self.P_ * self.C.T * S.I
    I = np.mat([[1, 0], [0, 1]])
    self.P = (I - self.G * self.C) * self.P_
    self.x = self.x_ + self.G * (Z - self.h(self.x_))

  # h(x_k)
  def h(self, x):
    '''[h(x_k)]
    
    Args:
        x ([type]): [description]
    
    Returns:
        [type]: [description]
    '''
    return np.mat([[self.h_(x, self.P_1)], [self.h_(x, self.P_2)], [self.h_(x, self.P_3)]])


  
  def h_(self, x, p):
    '''[推定距離の2乗の値を算出]
    
    Args:
        x ([list]): [事前状態（予測位置）]
        p ([list]): [基地局の座標]
    
    Returns:
        [float]: [推定距離の2乗]
    '''
    return (x[0, 0] - p[0, 0]) ** 2 + (x[1, 0] - p[1, 0]) ** 2


  def C_(self, x_):
    '''[ヤコビアン]
    
    入力値が2乗の場合のヤコビアン

    Args:
        x_ ([list]): [事前状態]
    
    Returns:
        [mat]: [ヤコビアンの計算結果]
    '''
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

  def getStatus(self):
    '''[状態情報（位置情報）の取得]
    
    Returns:
        [list]: [現在位置]
    '''
    return self.x
