import math
import numpy as np
import json
import pprint

class MapMaker():
    '''[六角形のグリッドを用いたマップマッチングのためのmap作成]
    '''

    filename = None  # fileの名前

    map_sizeX = []  # mapの大きさx[min, max]
    map_sizeY = []  # mapの大きさy[min, max]
    
    obstacle_size = []  # 障害物の大きさのリスト
    
    first_coodinate = None  # 初期グリッドの座標[x, y]
    grid_size = None  # グリッドの大きさ

    for_adjcent = []  # x軸方向のグリッドの個数
    map_list = []  # mapのグリッドリスト

    path = None  # 保存先のパス
    data = {}

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
            if cood[0] < self.map_sizeX[0]:
                # 右の場合は1
                cood = cood_back + np.array([int(self.grid_size * math.cos(pi / 3)), int(self.grid_size * math.sin(pi / 3))])
                self.for_adjcent.append([num, 1])
            else:
                # 左の場合は0
                self.for_adjcent.append([num, 0]) 

        self.map_list = map_list
        self.for_adjcent.append([None, None])

    def addObstacle(self, x, y):
        '''[障害物の追加]
        
        Args:
            x ([list]): [障害物のx座標(min, max)]
            y ([type]): [障害物のy座標(min, max)]
        '''
        sizedict = {}
        sizedict["x"] = x
        sizedict["y"] = y
        self.obstacle_size.append(sizedict)

    def printObstacle(self):
        pprint.pprint(self.obstacle_size)

    def makeJson(self):
        '''[json形式のデータを作成]

        保存形式(json)： [{"ID": id, "cood": coodinate, "adjacent": adjacentID} ...]

        Returns:
            [type]: [description]
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
                '''[listへの追加判定と追加]
                
                Args:
                    list ([list]): [追加するlist]
                    min ([int]): [追加できるグリッドの最小値]
                    max ([int]): [追加できるグリッドの最大値]
                    id ([int]): [判定するグリッドのID]
                
                Returns:
                    [list]: [判定後のlist]
                '''
                if min == None or max == None:
                    return False
                elif id >= min and id < max:
                    list.append(int(id))

                return

            list = []
            if back[2] == 0:
                addList(list, back[0], back[0] + back[1], now - now_size[1] - 1)
                addList(list, back[0], back[0] + back[1], now - now_size[1])
            elif back[2] == 1:
                addList(list, back[0], back[0] + back[1], now - now_size[1])
                addList(list, back[0], back[0] + back[1], now - now_size[1] + 1)

            addList(list, now_size[0], now_size[0] + now_size[1] - 1, now - 1)
            addList(list, now_size[0], now_size[0] + now_size[1] - 1, now)
            addList(list, now_size[0], now_size[0] + now_size[1] - 1, now + 1)

            if forward[2] == 0 and forward[1] != None:
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1])
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1] + 1)
            elif forward[2] == 1 and forward[1] != None:
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1] - 1)
                addList(list, forward[0], forward[0] + forward[1], now + now_size[1])

            return list

        def judgObstacle(cood):
            '''[障害物と重なっているグリッドを判定]
            
            Args:
                cood ([list]): [グリッドの座標]
            
            Returns:
                [boolean]: [障害物と重なっている場合はFalse, 重なっていない場合はTrue]
            '''
            for obstacle in self.obstacle_size:
                if cood[0] < obstacle['x'][0] or cood[0] > obstacle['x'][1]:
                    # 障害物のx座標と重なっていない場合
                    continue
                elif cood[1] < obstacle['y'][0] or cood[1] > obstacle['y'][1]:
                    # 障害物のy座標と重なっていない場合
                    continue
                else:
                    # 障害物と重なっている場合
                    return False
            return True

        i = 0

        # 列の始まりから現在までのグリッドの個数
        num = 0

        # 列の始まりのグリッドID
        firstID = 0

        # [最初のID, 列内のグリッドの個数, LorR]
        back = [None, None, None]

        # [最初のID, 列内のグリッドの個数, LorR]
        forward = []

        # [最初のID, 列内のグリッドの個数, LorR]
        size = [firstID, self.for_adjcent[i][0], self.for_adjcent[0][1]]

        for id, cood in enumerate(self.map_list):
            value = {}
            value["cood"] = cood
            if num < self.for_adjcent[i][0]:
                forward = [firstID + self.for_adjcent[i][0], self.for_adjcent[i + 1][0], self.for_adjcent[i][1]]
            else:
                back = size
                num = 0
                i += 1
                firstID = id
                size = [firstID, self.for_adjcent[i][0], self.for_adjcent[i][1]]
                forward = [firstID + self.for_adjcent[i][0], self.for_adjcent[i + 1][0], self.for_adjcent[i][1]]

            value["adjacent"] = adjJudg(id, back, forward, size)
            value["enable"] = judgObstacle(cood)
            self.data[id] = value
            num += 1

    def removeAdjList(self):
        for key in self.data.keys():
            adjlist = self.data[key]['adjacent']
            for adj in adjlist[:]:
                print(self.data[key])
                if self.data[adj]['enable'] == False:
                    adjlist.remove(adj)


    def saveFile(self, path, filename):
        '''[データのファイルへの保存]

        Args:
            path ([String]): [保存先の相対パス]
            filename ([String]): [保存するファイル名]
        '''
        self.path = path + filename

        with open(self.path, mode='w') as fw:
            fw.write(json.dumps(self.data))
