import math
import numpy as np
import json

class MapMaker():
    '''[六角形のグリッドを用いたマップマッチングのためのmap作成]
    '''

    filename = None  # fileの名前

    map_sizeX = []  # mapの大きさx[min, max]
    map_sizeY = []  # mapの大きさy[min, max]
    
    obstacle_size_x = []  # 障害物の大きさ[min, max]
    obstacle_size_y = []  # 障害物の大きさ[min, max]
    
    first_coodinate = None  # 初期グリッドの座標[x, y]
    grid_size = None  # グリッドの大きさ

    for_adjcent = []  # x軸方向のグリッドの個数
    map_list = []  # mapのグリッドリスト

    path = None  # 保存先のパス

    def __init__(self):
        '''[コンストラクタ]
        '''
        self.first_coodinate = np.array([0, 0])

    def setMapSize(self, x, y):
        '''[mapの大きさを設定するメソッド]
        
        Args:
            x ([list]): [mapのx座標(min, max)]
            y ([list]): [mapのy座標(min, max)]
        '''
        self.map_sizeX = x
        self.map_sizeY = y

    def addObstacle(self, x, y):
        '''[障害物の追加]
        
        Args:
            x ([list]): [障害物のx座標(min, max)]
            y ([type]): [障害物のy座標(min, max)]
        '''
        self.obstacle_size_x = x
        self.obstacle_size_y = y

    def setGridSize(self, size):
        '''[グリッドの大きさを設定]
        
        Args:
            size ([float]): [グリッドの大きさ]
        '''
        self.grid_size = size

    def gridDivision(self):
        '''[mapをグリッドに分割]
        '''
        map_list = []
        cood = self.first_coodinate
        pi = math.pi

        while cood[1] < self.map_sizeY[1]:
            num = 0
            for x in range(cood[0], self.map_sizeX[1], self.grid_size):
                num += 1
                temp_cood = [int(x), int(cood[1])]
                map_list.append(temp_cood)
            
            cood_back = np.array([cood[0], cood[1]])
            cood += np.array([int(-self.grid_size * math.cos(pi / 3)), int(self.grid_size * math.sin(pi / 3))])
            if cood[0] < self.map_sizeX[0]:  # 右の場合は1
                cood = cood_back + np.array([int(self.grid_size * math.cos(pi / 3)), int(self.grid_size * math.sin(pi / 3))])
                self.for_adjcent.append([num, 1])
            else:  # 左の場合は0
                self.for_adjcent.append([num, 0]) 

        self.map_list = map_list
        self.for_adjcent.append([None, None])

    def saveFile(self, path, filename):
        '''[データのファイルへの保存]
        保存形式(json)： [{"ID": id, "cood": coodinate, "adjacent": adjacentID} ...]

        Args:
            path ([String]): [保存先の相対パス]
            filename ([String]): [保存するファイル名]
        '''
        def adjJudg(now, back, forward, now_size):
            '''[グリッドの隣接判定]
            
            Args:
                now ([int]): [現在のグリッドID]
                back ([list]): [前のグリッド列の情報]
                forward ([list]): [次のグリッド列の情報]
                now_size ([list]): [現在のグリッド列の情報]
            
            Returns:
                [list]: [mapのグリッドリスト]
            '''
            
            def addList(list, min, max, id):
                '''[listへの追加判定]
                
                Args:
                    list ([list]): [追加するlist]
                    min ([int]): [追加できるグリッドの最小値]
                    max ([int]): [追加できるグリッドの最大値]
                    id ([int]): [判定するグリッドのID]
                
                Returns:
                    [list]: [判定後のlist]
                '''
                if min == None or max == None:
                    return

                if id >= min and id < max:
                    list.append(int(id))

                return list

            list = []
            if back[2] == 0:
                addList(list, back[0], back[0] + back[1], now - now_size[1] - 1)
                addList(list, back[0], back[0] + back[1], now - now_size[1])
            elif back[2] == 1:
                addList(list, back[0], back[0] + back[1], now - now_size[1])
                addList(list, back[0], back[0] + back[1], now - now_size[1] + 1)

            addList(list, now_size[0], now_size[0] + now_size[1] - 1, now - 1)
            addList(list, now_size[0], now_size[0] + now_size[1] - 1, now + 1)
                    
            if forward[2] == 0 and forward[1] != None:
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1])
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1] + 1)
            elif forward[2] == 1 and forward[1] != None:
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1] - 1)
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1])
                    
            return list

        self.path = path + filename

        grid_list = []
        id = 0  # 現在のグリッドの個数
        i = 0
        num = 0  # 列の始まりから現在までのグリッドの個数
        firstID = 0  # 列の始まりのグリッドID
        back = [None, None, None] # [最初のID, 列内のグリッドの個数, LorR]
        forward = []  # [最初のID, 列内のグリッドの個数, LorR]
        size = [firstID, self.for_adjcent[i][0], self.for_adjcent[0][1]]  # [最初のID, 列内のグリッドの個数, LorR]

        for list in self.map_list:
            data = {}
            data["ID"] = int(id)
            data["cood"] = list
            if num < self.for_adjcent[i][0]:
                forward = [firstID + self.for_adjcent[i][0], self.for_adjcent[i + 1][0], self.for_adjcent[i][1]]
                data["adjacent"] = adjJudg(id, back, forward, size)
            else:
                back = size
                num = 0
                i += 1
                firstID = id
                size = [firstID, self.for_adjcent[i][0], self.for_adjcent[i][1]]
                forward = [firstID + self.for_adjcent[i][0], self.for_adjcent[i + 1][0], self.for_adjcent[i][1]]
                data["adjacent"] = adjJudg(id, back, forward, size)

            grid_list.append(data)
            num += 1
            id += 1

        with open(self.path, mode='w') as fw:
            fw.write(json.dumps(grid_list))