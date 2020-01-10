import math
import numpy as np
from scipy.stats import norm
import json

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

    # 隣接しているグリッド
    grid_list = None
    pri_grid_list = None

    # マップ情報
    map_data = None
    now_grid = None

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

    # 尤度計算のモードの切り替え(ガウス分布=0, ベイズ推定量=1)
    mode = 0

    
    def __init__(self, now_grid, p_i, sigma_range):
        '''[コンストラクタ]
        
        Args:
            p_i ([list]): [基地局の座標]
            sigma_range ([float]): [観測値の分散]
        '''
        self.now_grid = now_grid
        self.p_i = p_i
        self.sigma_range = sigma_range
        self.pri_grid_list = [self.now_grid]

    
    def readMap(self, path):
        '''[マップ情報の読み込み]
        
        Args:
            filename ([String]): [jsonファイルのpath]
        '''
        with open(path, mode='r') as fr:
            self.map_data = json.loads(fr.readline())
            

    # 予測ステップ
    def prediction(self):
        '''[予測ステップ]

        1. 現在のグリッド情報の読み込み
        2. 事前確率の算出
        3. 予測確率の算出
        '''

        def readGridInfo(map_data, grid):
            '''[現在のグリッド情報の読み込み]
            
            Args:
                map_data ([dict]): [読み込んだjsonファイルの中のマップ情報]
                grid ([Integer]): [現在のグリッド]
            
            Returns:
                cood ([list]): [座標]
                adj ([list]): [隣接しているグリッドのID]
            '''
            info_dict = map_data.get(str(grid))
            cood = info_dict.get('cood')
            adj = info_dict.get('adjacent')

            return cood, adj

        def priPro(grid_list, map_data, pri_grid_list, P_x_n):
            '''[事前確率の算出]
            
            Args:
                grid_list ([list]): [隣接しているグリッドリスト]
                map_data ([dict]): [マップ情報]
                pri_grid_list ([list]): [前回隣接していたグリッドリスト]
                P_x_n ([mat]): [前回の事後確率]
            
            Returns:
                [mat]: [事前確率]
            '''
            P = []

            for grid in grid_list:
                info_dict = map_data.get(str(grid))
                adj = info_dict.get('adjacent')
                TorF = []
                # グリッド候補(grid_lsit)の隣接グリッド(adj)に前回のグリッド候補(pri_grid_list)が存在しているかどうかのチェック
                for a in pri_grid_list:
                    if a in adj:
                        TorF.append(1)
                    else:
                        TorF.append(0)
                mat = np.mat([TorF])  # 存在しているグリッドのTorFの行列(1)
                adam = np.multiply(mat, P_x_n)  # 行列(1)と前回の事後確率とのアダマール積の行列(2)
                P.append(np.sum(adam))  # 行列(2)内の値の総和

            return np.mat([P])

        def predictPro(P_):
            '''[予測確率]
            
            Args:
                P_ ([mat]): [事前確率]
            
            Returns:
                [mat]: [予測確率]
            '''
            P = []
            for i in range(P_.size):
                P.append(1/7)
            P_motion = np.mat([P])

            return np.multiply(P_, P_motion)


        cood, adj = readGridInfo(self.map_data, self.now_grid)
        self.grid_list = adj
        P_ = priPro(self.grid_list, self.map_data, self.pri_grid_list, self.P_x_n)
        self.P_x_n_ = predictPro(P_)


    # 更新ステップ
    def update(self, y_n):
        '''[更新ステップ]
        
        1. 観測値から各候補位置の尤度を算出
        2. 観測値による尤度と各位置の予測確率を用いて各グリッドの事後確率を算出
        3. 候補位置の確率の正規化
        4. 正規化した確率の中から最も高い確率のグリッドの中心を推定位置に決定
        Args:
            y_n ([list]): [観測値]
        
        Returns:
            [list]: [現在の予測結果]
        '''

        def setCoodList(map_data, grid_list):
            '''[グリッド候補の座標リストの作成]
            
            Args:
                map_data ([dict]): [マップ情報]
                grid_list ([list]): [グリッドIDのリスト]
            
            Returns:
                [matrix]: [グリッド候補の座標]
            '''
            
            cood_list = []
            for gridID in grid_list:
                info = map_data.get(str(gridID))  # グリッド情報を取得
                cood = info.get('cood') # 座標の取得
                cood_list.append(np.mat(cood))

            #print(cood_list)

            return cood_list

        def observe(x, y_n, p_i, scale_range):
            '''[観測値からグリッドの尤度を計算]

            pdf : 観測結果による確率密度
            loc : ガウス分布の期待値
            scale_range : 標準偏差
            
            Args:
                x ([list]): [グリッドの座標]
                y_n ([list]): [観測値]
                p_i ([list]): [基地局の座標]
                scale_range ([float]): [標準偏差]
            
            Returns:
                [float]: [観測結果による確率密度]
            '''

            # 基地局と位置の候補との距離を計算
            def h(x, p_i):
                h_1 = ((x[0, 0] - p_i[0, 0]) ** 2 + (x[0, 1] - p_i[0, 1]) ** 2) ** 0.5
                h_2 = ((x[0, 0] - p_i[1, 0]) ** 2 + (x[0, 1] - p_i[1, 1]) ** 2) ** 0.5
                h_3 = ((x[0, 0] - p_i[0, 2]) ** 2 + (x[0, 1] - p_i[1, 2]) ** 2) ** 0.5

                return np.mat([[h_1], [h_2], [h_3]])

            n = []
            loc = h(x, p_i)
            for num in range(3):
                # 正規分布の尤度計算（対数への変換）
                l = -1/2*math.log(2*math.pi)-1/2*math.log(scale_range)-1/(2*scale_range**2)*(y_n[num, 0]-loc[num, 0])**2
                n.append(l)

            pdf = n[0] * n[1] * n[2]
            return pdf

        def bayesLisk(x, y_n, p_i):
            '''[観測値からベイズ推定量を計算]
            
            Args:
                x ([list]): [グリッドの座標]
                y_n ([list]): [観測値]
                p_i ([list]): [基地局の座標]
            
            Returns:
                [float]: [ベイズリスク]
            '''

            # 基地局と位置の候補との距離を計算
            def h(x, p_i):
                h_1 = ((x[0, 0] - p_i[0, 0]) ** 2 + (x[0, 1] - p_i[0, 1]) ** 2) ** 0.5
                h_2 = ((x[0, 0] - p_i[1, 0]) ** 2 + (x[0, 1] - p_i[1, 1]) ** 2) ** 0.5
                h_3 = ((x[0, 0] - p_i[0, 2]) ** 2 + (x[0, 1] - p_i[1, 2]) ** 2) ** 0.5

                return np.mat([[h_1], [h_2], [h_3]])

            '''
            def errorToLike(error):
                max_error = error.max()
                min_error = error.min()
                deno = max_error - min_error  # 分母
                max = 1
                min = 0.3
                norm = max - min
                
                pdf = 1
                for num in range(len(error)):
                    like = (error[num, 0] - min_error) / deno

                    print("like:" + str(like))
                    pdf *= like
                    
                return pdf'''

            # 観測値とh()との2乗誤差
            dif = y_n - h(x, p_i)
            square_error = np.multiply(dif, dif)

            # ベイズ推定量の算出
            error_sum = 0
            for num in range(len(square_error)):
                error_sum += square_error[num, 0]
            bayes_lisk = error_sum / len(square_error)

            return bayes_lisk

        def errorToLike(lisk):
            '''[ベイズリスクから存在確率を算出]
            
            Args:
                lisk ([list]): [ベイズリスク]
            
            Returns:
                [list]: [グリッド毎の存在確率]
            '''
            
            max_error = max(lisk)
            min_error = min(lisk)
            deno = max_error - min_error  # 分母

            like = []
            for num in range(len(lisk)):
                like.append((lisk[num] - min_error) / deno)

            print("like:" + str(like))

            return like

        def likeNormal(like):
            '''[尤度の正規化]
            
            最大値を1、最小値を0にする正規化
            Args:
                like ([list]): [尤度のリスト]
            
            Returns:
                [matrix]: [正規化した尤度の行列]
            '''
            maxlike = max(like)
            minlike = min(like)

            likelihood = []
            for l in like:
                ll = (l - minlike) / (maxlike - minlike)
                likelihood.append(ll)

            print(likelihood)

            return np.mat(likelihood)

        def likeToPro(P_x_n_, like):
            '''[尤度を元に各グリッドの事後確率を計算]
            
            Args:
                P_x_n_ ([list]): [予測確率]
                like ([list]): [正規化された尤度]
            
            Returns:
                [list]: [事後確率]
            '''
            P_x_n = np.multiply(P_x_n_, like)

            return P_x_n
                
        def proNormal(P):
            '''[確率の総和を1にする正規化]

            Args:
                P ([list]): [確率のリスト]
            
            Returns:
                [list]: [正規化した確率]
            '''
            pro = P.tolist()[0]
            sum = 0
            for p in pro:
                sum += p

            P_normal = P / sum
            
            return P_normal

        def selectX(cood_list, grid_list, P_x_n):
            '''[位置の候補の内，最も確率の高いグリッドを選択]
            
            Args:
                cood_list ([list]): [グリッド候補の座標のリスト]
                grid_list ([list]): [グリッド候補のリスト]
                P_x_n ([matrix]): [事後確率]
            
            Returns:
                [matrix]: [最も確率の高いグリッドの座標]
                [Integer]: [最も確率の高いグリッドID]
            '''

            max = np.argmax(P_x_n)
            
            return cood_list[max], grid_list[max]

        # 座標候補リストの作成
        self.cood_list = setCoodList(self.map_data, self.grid_list)

        like = []
        if self.mode == 0:
            # ガウス分布を用いた尤度計算
            for num in range(len(self.cood_list)):
                like.append(observe(self.cood_list[num], y_n, self.p_i, self.sigma_range))    
            # 尤度の正規化
            like = likeNormal(like)

        else:
            # ベイズリスクの算出
            bayes_lisk = []
            for num in range(len(self.cood_list)):
                bayes_lisk.append(-bayesLisk(self.cood_list[num], y_n, self.p_i))
            # ベイズ推定量を用いた尤度計算
            like = errorToLike(bayes_lisk)
        print(like)

        # 確率の計算と確率の正規化
        self.P_x_n = likeToPro(self.P_x_n_, like)
        self.P_x_n = proNormal(self.P_x_n)
        #print("pro : " + str(self.P_x_n))

        # 最も確率の高いグリッドの選択
        self.x_n, self.now_grid = selectX(self.cood_list, self.grid_list, self.P_x_n)
        self.x_n = self.x_n.T
        self.pri_grid_list = self.grid_list
        self.pri_cood_list = self.cood_list

        return self.x_n, self.now_grid
