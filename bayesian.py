import math
import numpy as np
from scipy.stats import norm

class BaysianFilter:
    '''
    使い方
    1. インスタンス作成時に、初期位置・移動半径・基地局の座標・観測値の分散を引数として入力
    2. readMap（マップ情報の取得）を呼び出す
    2. prediction（予測ステップ）を呼び出す
    3. update（更新ステップ）を呼び出す
    '''

    # 移動場所の候補
    cood_list = None
    pri_cood_list = None

    # マップ情報
    x_min = None
    x_max = None
    y_min = None
    y_max = None

    # システムモデル
    x_n = None
    c = None

    # 観測モデル
    y_n = None
    p_i = None
    sigma_range = None

    # 予測式
    P_x_n_ = [1]
    P_motion = []
    motion = 1 / 7
    
    # 更新式
    P_x_n = [1]
    P_y_n = []

    
    # コンストラクタ
    def __init__(self, x, c, p_i, sigma_range):
        '''
        引数は以下の４つ
        ・x：初期位置
        ・c：移動半径（グリッドの大きさを決定）
        ・p_i：基地局の座標
        ・sigma_range：観測値の分散
        '''
        self.x_n = x
        self.c = c
        self.p_i = p_i
        self.sigma_range = sigma_range
        self.pri_cood_list = np.mat([[x[0, 0], x[1, 0]]])

    
    # マップ情報の読み込み
    def readMap(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max


    # 予測ステップ
    def prediction(self):
        '''
        1. t=n+1の時のグリッド候補の決定
        2. グリッド候補が移動可能か判定（マップマッチング）
        3. t=nの時のグリッド候補がt=n+1の座標に隣接しているかの判定
        4. 予測確率の算出
        '''
    
        # 次のグリッド候補の決定
        def coodinate(x_n, c):
            '''
            座標は真ん中が0, 右上から時計回りに1,2,3,4,5,6と番号を割り振り
            0は座標が変わらない
            2,5は右に移動半径分進んだ位置
            1,3,4,6は移動半径にsin, cosを用いて位置を決定
            '''
            cood_list = []
            pi = math.pi
            x = c * math.cos(pi / 3)
            y = c * math.sin(pi / 3)

            cood_list.append(x_n.T)                                                               # 0
            cood_list.append(np.mat([[math.floor(x_n[0, 0] + x), math.floor(x_n[1, 0] + y)]]))    # 1
            cood_list.append(np.mat([[math.floor(x_n[0, 0] + c), math.floor(x_n[1, 0])]]))        # 2
            cood_list.append(np.mat([[math.floor(x_n[0, 0] + x), math.floor(x_n[1, 0] - y)]]))    # 3
            cood_list.append(np.mat([[math.floor(x_n[0, 0] - x), math.floor(x_n[1, 0] - y)]]))    # 4
            cood_list.append(np.mat([[math.floor(x_n[0, 0] - c), math.floor(x_n[1, 0])]]))        # 5
            cood_list.append(np.mat([[math.floor(x_n[0, 0] - x), math.floor(x_n[1, 0] + y)]]))    # 6

            return cood_list

        # マップマッチング
        def mapMatching(cood_list, P_x_n_):
            P = []
            for num in range(len(cood_list)):
                if cood_list[num][0, 0] < self.x_min or cood_list[num][0, 0] > self.x_max:
                    P.append(0)
                elif cood_list[num][1, 0] < self.y_min or cood_list[num][1, 0] > self.y_max:
                    P.append(0)
                else:
                    P.append(P_x_n_[num])

            return P

        # 隣接している六角形の判定
        def adjacent(x, pri_cood_list, c, motion):
            '''
            引数
            ・x: 移動候補の六角形のグリッドの一つの座標
            ・t=nのグリッドの候補
            六角形の中心座標間の距離が移動半径と（ほぼ）等しい時隣接していると判定→motionの値をリストに追加
            移動半径と等しくない時は隣接していないと判定→0をリストに追加
            '''
            list = []
            for num in range(len(pri_cood_list)):
                dis = math.sqrt((x[0, 0] - pri_cood_list[num][0, 0]) ** 2 + (x[0, 1] - pri_cood_list[num][0, 1]) ** 2)
                if dis < c + 10 and dis > c - 10 or dis == 0:
                    list.append(motion)
                else:
                    list.append(0)
            P_motion = list

            return P_motion

        # 前回のそれぞれの位置から動いてきた確率の合計値（予測確率）を算出
        def priPro(P_motion, P_):
            P = 0
            for num in range(len(P_motion)):
                P += P_motion[num] * P_[num]

            return P

        list = []
        self.cood_list = coodinate(self.x_n, self.c)
        #self.P_x_n_ = mapMatching(self.cood_list, self.P_x_n_) #マップマッチング機能は未実装

        for num in range(len(self.cood_list)):
            self.P_motion = adjacent(self.cood_list[num], self.pri_cood_list, self.c, self.motion)
            P = priPro(self.P_motion, self.P_x_n)
            list.append(P)

        self.P_x_n_ = list
        print(list)


    # 更新ステップ
    def update(self, y_n):
        '''
        1. 観測値から各候補位置の尤度を算出
        2. 観測値による尤度と各位置の予測確率を用いて各グリッドの事後確率を算出
        3. 候補位置の確率の正規化
        4. 正規化した確率の中から最も高い確率のグリッドの中心を推定位置に決定
        '''

        # 観測値から各位置の尤度を計算
        def observe(x, y_n, p_i, scale_range):
            '''
            pdf : 観測結果による確率密度
            loc : ガウス分布の期待値
            scale_range : 標準偏差
            '''

            # 基地局と位置の候補との距離を計算
            def h(x, p_i):
                h_1 = (x[0, 0] - p_i[0, 0]) ** 2 + (x[0, 1] - p_i[0, 1]) ** 2
                h_2 = (x[0, 0] - p_i[1, 0]) ** 2 + (x[0, 1] - p_i[1, 1]) ** 2
                h_3 = (x[0, 0] - p_i[0, 2]) ** 2 + (x[0, 1] - p_i[1, 2]) ** 2

                return np.mat([[h_1], [h_2], [h_3]])

            n = []
            loc = h(x, p_i)
            for num in range(3):
                l = -1/2*math.log(2*math.pi)-1/2*math.log(scale_range)-1/(2*scale_range**2)*(y_n[num, 0]-loc[num, 0])**2
                n.append(l)

            pdf = n[0] * n[1] * n[2]
            return pdf

         #　尤度の正規化
        def likeNormal(like):
            '''
            最大値を1、最小値を0にする正規化
            ・引数
                like : 尤度のリスト
            ・返り値
                likelihood : 正規化した尤度のリスト
            '''
            maxlike = max(like)
            minlike = min(like)

            likelihood = []
            for l in like:
                ll = (l - minlike) / (maxlike - minlike)
                likelihood.append(ll)

            return likelihood

        # 尤度を元に各グリッドの事後確率を計算
        def likeToPro(P_x_n_, like):
            '''
            ・引数
                P_x_n_ : 予測確率
                like : 正規化された尤度
            ・返り値
                P_x_n : 事後確率
            '''
            P_x_n = []
            for num in range(len(P_x_n_)):
                P_x_n.append(P_x_n_[num] * like[num])

            return P_x_n
                
        # 確率の正規化
        def proNormal(P):
            '''
            確率の総和を1にする正規化
            ・引数
                P : 確率のリスト
            ・返り値
                P_new : 正規化した確率
            '''
            sum = 0
            for num in range(len(P)):
                sum += P[num]

            P_normal = []
            for p in P:
                P_normal.append(p / sum)
            
            return P_normal

        # 位置の候補の内，最も確率の高い位置を選択
        def selectX(x, P_x_n):
            max = np.argmax(P_x_n)
            
            return x[max]

        like = []
        for num in range(len(self.cood_list)):
            like.append(observe(self.cood_list[num], y_n, self.p_i, self.sigma_range))
        
        like = likeNormal(like)
        self.P_x_n = likeToPro(self.P_x_n_, like)
        print("pro : " + str(self.P_x_n))
        self.P_x_n = proNormal(self.P_x_n)
        print(self.P_x_n)
        
        self.x_n = selectX(self.cood_list, self.P_x_n)

        return self.x_n