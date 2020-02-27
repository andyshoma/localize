import bayesian
import kalman
import numpy as np
import json

class BKmixture:

    now_cood = None
    station = None

    # Kalman Filterのパラメータ
    kf = None

    # Bayesian Filterのパラメータ
    bf = None
    station = None
    sigma_range = None
    path = None

    # マップデータ
    map_data = None

    # 観測値
    Z = None

    # Kalman Filterの推定位置
    x_kalman = None

    # Bayesian Filterの推定位置
    x_bayesian = None
    grid = None

    map_x = [[1330, 3130], [3830, 5630], [6330, 8130], [8830, 10630]]
    map_y = [[6340, 7540], [10090, 11290], [13090, 14290], [2590, 3190]]

    def __init__(self, station):
        '''[コンストラクタ]
        
        Args:
            station ([mat]): [各基地局の座標]
        '''
        self.station = station

    def setKalman(self, now_cood, p_1, p_2, p_3, o_walk, o_range, r):
        '''[Kalman Filterのパラメータ設定]
        
        Args:
            now_cood ([list]): [現在地の座標]
            p_1 ([mat]): [基地局1の座標]
            p_2 ([mat]): [基地局2の座標]
            p_3 ([mat]): [基地局3の座標]
            o_walk ([int]): [状態モデルの誤差分散]
            o_range ([int]): [観測モデルの誤差分散]
        '''
        self.kf = kalman.KalmanFilter(now_cood[0], now_cood[1], p_1, p_2, p_3)
        self.kf.setModel(o_walk, o_range, r)

    def setBayesian(self, sigma_range, path):
        '''[Bayesian Filterのパラメータ設定]
        
        Args:
            sigma_range ([int]): [FTMの誤差分散]
            path ([string]): [マップ情報ファイルのパス]
        '''
        self.sigma_range = sigma_range
        self.path = path

        # マップデータの読み込み
        with open(path, mode='r') as fr:
            self.map_data = json.loads(fr.readline())

    def estimate(self, Z):
        '''[位置推定]
        
        Args:
            Z ([mat]): [観測値]
        
        Returns:
            [type]: [description]
        '''
        def kalmanStep(Z):
            '''[Kalman Filterの推定ステップ]
            
            Args:
                Z ([mat]): [観測値]
            
            Returns:
                [list]: [Kalman Filterの推定値]
            '''
            self.kf.estimate()
            self.kf.filter(np.square(Z))
            x_kalman = self.kf.getStatus()

            return x_kalman

        def isExistX(cood):
            for x in self.map_x:
                if cood[0, 0] > x[0] and cood[0, 0] < x[1]:
                    return True
            return False

        def isExistY(cood):
            for y in self.map_y:
                if cood[1, 0] > y[0] and cood[1, 0] < y[1]:
                    return True
            return False
            
        def search(x_kalman):
            '''[Kalman Filterによる推定座標に近いグリッドの総当たり探索]
            
            Args:
                x_kalman ([list]): [Kalman Filterの推定値]
            
            Returns:
                [int]: [グリッドのID]
            '''
            mindis = 1000000
            minkey = 0
            for key in self.map_data.keys():
                x_cood = self.map_data[key]['cood'][0]
                y_cood = self.map_data[key]['cood'][1]
                # 各グリッドの中心座標とのユークリッド距離を算出
                dis = ((x_kalman[0, 0] - x_cood) ** 2 + (x_kalman[1, 0] - y_cood) ** 2) ** 0.5
                if dis < mindis:
                    # 最も小さいユークリッド距離のkeyを探索
                    minkey = key
                    mindis = dis

            now_gridID = minkey
            return now_gridID

        def bayesianStep(now_gridID, Z):
            '''[Bayesian Filterの推定ステップ]
            
            Args:
                now_gridID ([int]): [Kalman Filterの推定値に最も近いグリッドのID]
                Z ([mat]): [観測値]
            
            Returns:
                [list]: [推定位置]
                [int]: [推定グリッド]
            '''
            self.bf = bayesian.BaysianFilter(now_gridID, self.station, self.sigma_range)
            self.bf.readMap(self.path)
            self.bf.prediction()
            x_bayesian, grid = self.bf.update(Z)

            return x_bayesian, grid
            
        x_kalamn = kalmanStep(Z)
        if isExistX(x_kalamn) and isExistY(x_kalamn):
            now_gridID = search(x_kalamn)
            x, grid = bayesianStep(now_gridID, Z)
        else:
            x = x_kalamn
            grid = -10

        return x, grid
