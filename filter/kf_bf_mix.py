from filter.new_kalman import *
from filter.new_bayesian import *
from world import *


class KalmanBayesianMixture:
    def __init__(self, envmap, init_pose, path, motion_noise_stds=np.diag([1000000, 1000000]), distance_dev_rate=300):
        self.map = envmap
        self.pose = init_pose
        self.motion_noise_stds = motion_noise_stds
        self.distance_dev_rate = distance_dev_rate
        self.path = path
        self.kf = Kalman(envmap, init_pose, motion_noise_stds=motion_noise_stds, distance_dev_rate=distance_dev_rate)
        self.poses = [self.pose]
        self.enable = True
        self.position = 0

        self.map_data = read_map(path)

    def motion_update(self):
        self.kf.motion_update()

    def observation_update(self, observation):
        self.kf.observation_update(observation)
        self.pose = self.kf.pose

        self.grid_id = search(self.path, self.pose)
        data = read_grid_info(self.map_data, self.grid_id)
        self.enable = data[2]
        if self.enable == False:
            self.pose = data[0]
            self.adjacent = data[1]
            self.bf = Bayesian(self.map, init_grid=Grid(self.grid_id, self.pose), path=self.path, distance_dev_rate=self.distance_dev_rate)
            self.bf.motion_update()
            self.bf.observation_update(observation)

    def draw(self, ax, elems):
        if self.enable == True:
            self.draw_kalman(ax, elems)
        else:
            self.draw_bayesian(ax, elems)

    def draw_kalman(self, ax, elems):
        self.belief = self.kf.belief
        e = sigma_ellipse(self.belief.mean[0:2], self.belief.cov[0:2, 0:2], 3)
        elems.append(ax.add_patch(e))

        x, y = self.belief.mean
        c = patches.Circle(xy=(x, y), radius=100, fill=True, color="red")
        elems.append(ax.add_patch(c))
        elems += ax.plot([self.pose[0], self.map.rps[(self.position-1)%8].pos[0]], [self.pose[1], self.map.rps[(self.position-1)%8].pos[1]], color="pink")

        if len(self.poses) >= 9:
            self.poses.pop(-9)
        self.poses.append(self.pose)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")
        self.position += 1

    def draw_bayesian(self, ax, elems):
        x = self.bf.grid.cood[0]
        y = self.bf.grid.cood[1]
        c = patches.Circle(xy=(x, y), radius=100, fill=True, color="red")
        elems.append(ax.add_patch(c))
        elems += ax.plot([x, self.map.rps[(self.position - 1) % 8].pos[0]],
                         [y, self.map.rps[(self.position - 1) % 8].pos[1]], color="pink")

        xs = [adj.cood[0] for adj in self.adjacent]
        ys = [adj.cood[1] for adj in self.adjacent]
        vxs = [adj.weight * len(self.adjacent) * 1000 for adj in self.adjacent]
        vys = [adj.weight * len(self.adjacent) * 1000 for adj in self.adjacent]
        elems.append(ax.quiver(xs, ys, vxs, vys,
                               angles='xy', scale_units='xy', scale=1, color="blue", alpha=0.5))

        if len(self.poses) >= 9:
            self.poses.pop(-9)
        self.poses.append(self.bf.grid.cood)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")
        self.position += 1


if __name__ == '__main__':
    world = World(100, 2)

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

    path = "/Users/andy/Wifi_rtt/localize/sample.json"

    init_pose = np.array([3330,8940]).T
    filter = KalmanBayesianMixture(m, init_pose, path)
    pedestrian = Pedestrian(pose=np.array([20,20]), estimator=filter, color="red")
    world.append(pedestrian)
    world.append(m)

    world.draw()