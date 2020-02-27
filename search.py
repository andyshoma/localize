import math
import json

class Search():
    '''[summary]
    '''
    map_data = None
    e = None

    def __init__(self):
        '''[summary]
        '''
        self.e = "jey"

    def readJsonFile(self, path):
        with open(path, mode='r') as fr:
            self.map_data = json.load(fr)

    def searching(self, cood):
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
            dis = ((cood[0] - x_cood) ** 2 + (cood[1] - y_cood) ** 2) ** 0.5
            if dis < mindis:
                # 最も小さいユークリッド距離のkeyを探索
                minkey = key
                mindis =  dis

        now_gridID = minkey
        return now_gridID
