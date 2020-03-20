import json
from scipy.stats import norm
from new_kalman import*
from world import*


def read_map(path):
    with open(path, mode='r') as fr:
        map_data = json.loads(fr.readline())

    return map_data


def read_grid_info(map_data, grid_num):
    info = map_data.get(str(grid_num))
    coodinate = info.get('cood')
    adjacent = [Grid(id, map_data.get(str(id)).get('cood')) for id in info.get('adjacent')]

    return coodinate, adjacent


def prior_probability(adjacent_grid, map_data):
    # 事前確率の計算（今はしない）
    pass


def predict_provability(grid_len):
    pro = 1/grid_len

    return pro

def max_adjacent(adjacent):
    max_grid = Grid(id=None, cood=None, weight=0)
    for grid in adjacent:
        if grid.weight > max_grid.weight: max_grid = grid

    return max_grid


def search(path, cood):
    with open(path, mode='r') as fr:
        map_data = json.load(fr)

    mindis = 1000000
    minkey = 0
    for key in map_data.keys():
        x_cood = map_data[key]['cood'][0]
        y_cood = map_data[key]['cood'][1]
        # 各グリッドの中心座標とのユークリッド距離を算出
        dis = ((cood[0] - x_cood) ** 2 + (cood[1] - y_cood) ** 2) ** 0.5
        if dis < mindis:
            # 最も小さいユークリッド距離のkeyを探索
            minkey = key
            mindis = dis

    return minkey



class Grid:
    def __init__(self, id, cood, weight=1):
        self.id = id
        self.cood = cood
        self.weight = weight

    def motion_update(self, grid_len):
        self.weight = predict_provability(grid_len)

    def observation_update(self, observation, envmap, distance_dev_rate):
        for id in observation:
            z = observation[id][0]
            ap_pos = None
            for ap in envmap.aps:
                if ap.id == id: ap_pos = ap.pos

            # グリッドの位置と地図からランドマークの距離を算出
            grid_suggest_pos = observation_function(self.cood, ap_pos)

            # 尤度の計算
            self.weight *= norm.pdf(z, loc=grid_suggest_pos, scale=distance_dev_rate)


class Bayesian:

    GAUSIAN = 1                 # ガウス分布の尤度を利用
    BAYESIAN_ESTIMATION = 2     # ベイズ推定量を利用

    def __init__(self, envmap, init_grid, path, distance_dev_rate=1200, mode=GAUSIAN):
        '''Bayesian Filter

        Args:
            envmap: アクセスポイントのマップ
            init_grid: 初期姿勢
            path: 地図情報ファイルの絶対パス
            distance_dev_rate: 測距結果の標準偏差
            mode: 尤度orベイズ推定量
        '''
        self.map = envmap
        self.grid = init_grid
        self.distance_dev_rate = distance_dev_rate
        self.mode = mode
        self.adjacent = []
        self.poses = [self.grid.cood]
        self.position = 0

        self.map_data = read_map(path)

    def motion_update(self):
        self.grid.cood, self.adjacent = read_grid_info(self.map_data, self.grid.id)
        for adj in self.adjacent: adj.motion_update(len(self.adjacent))

    def observation_update(self, observation):
        for adj in self.adjacent: adj.observation_update(observation, self.map, self.distance_dev_rate)
        self.grid = max_adjacent(self.adjacent)
        self.normalize()

    def normalize(self):
        sum = 0
        for adj in self.adjacent: sum += adj.weight
        for adj in self.adjacent: adj.weight = adj.weight/sum

    def draw(self, ax, elems):
        x = self.grid.cood[0]
        y = self.grid.cood[1]
        c = patches.Circle(xy=(x, y), radius=100, fill=True, color="red")
        elems.append(ax.add_patch(c))
        elems += ax.plot([x, self.map.rps[(self.position - 1) % 8].pos[0]],
                         [y, self.map.rps[(self.position - 1) % 8].pos[1]], color="pink")

        xs = [adj.cood[0] for adj in self.adjacent]
        ys = [adj.cood[1] for adj in self.adjacent]
        vxs = [adj.weight*len(self.adjacent)*1000 for adj in self.adjacent]
        vys = [adj.weight*len(self.adjacent)*1000 for adj in self.adjacent]
        elems.append(ax.quiver(xs, ys, vxs, vys,
                               angles='xy', scale_units='xy', scale=1, color="blue", alpha=0.5))

        if len(self.poses) >= 9:
            self.poses.pop(-9)
        self.poses.append(self.grid.cood)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")
        self.position += 1


class Bayesian_Pedestrian(Pedestrian):
    def __init__(self, pose, estimator=None, sensor=None, color="black"):
        super().__init__(pose, estimator, sensor, color)

    def one_step(self, time_interval, i):
        for num in range(7):
            self.estimator.motion_update()
            self.estimator.observation_update(self.observations[i])


if __name__ == '__main__':
    world = World(100, 0.5)

    # 地図を生成して3つランドマークを追加 #
    m = Map()
    m.appendAp(AccessPoint(9330, 12940, 200372))
    m.appendAp(AccessPoint(9330, 2940, 200359))
    m.appendAp(AccessPoint(1830, 6940, 200358))

    m.appendRp(ReferencePoint(3330, 8940, 1))
    m.appendRp(ReferencePoint(3330, 11940, 2))
    m.appendRp(ReferencePoint(5830, 11940, 3))
    m.appendRp(ReferencePoint(8330, 11940, 4))
    m.appendRp(ReferencePoint(8330, 8940, 5))
    m.appendRp(ReferencePoint(8330, 4440, 7))
    m.appendRp(ReferencePoint(5830, 4440, 8))
    m.appendRp(ReferencePoint(3330, 4440, 9))

    m.appendObstacle(Obstacle(1330, 6340, 1800, 1200))
    m.appendObstacle(Obstacle(1330, 10090, 1800, 1200))
    m.appendObstacle(Obstacle(1330, 12850, 1800, 1200))
    m.appendObstacle(Obstacle(3830, 6340, 1800, 1200))
    m.appendObstacle(Obstacle(3830, 10090, 1800, 1200))
    m.appendObstacle(Obstacle(3830, 12850, 1800, 1200))
    m.appendObstacle(Obstacle(6330, 6340, 1800, 1200))
    m.appendObstacle(Obstacle(6330, 10090, 1800, 1200))
    m.appendObstacle(Obstacle(6330, 12850, 1800, 1200))
    m.appendObstacle(Obstacle(6330, 2590, 1800, 600))
    m.appendObstacle(Obstacle(8830, 6340, 1800, 1200))
    m.appendObstacle(Obstacle(8830, 10090, 1800, 1200))
    m.appendObstacle(Obstacle(8830, 12850, 1800, 1200))
    m.appendObstacle(Obstacle(8830, 2590, 1800, 600))

    path = "/Users/andy/Wifi_rtt/localize/map_07m_true.json"

    init_pose = np.array([3330,8940]).T
    init_grid_id = search(path, init_pose)
    bf = Bayesian(m, init_grid=Grid(init_grid_id, init_pose), path=path)
    pedestrian = Bayesian_Pedestrian(pose=np.array([20,20]), estimator=bf, color="red")
    world.append(pedestrian)
    world.append(m)

    world.draw()

