import numpy as np
import math
from scipy.stats import multivariate_normal
from matplotlib.patches import Ellipse
from matplotlib import patches
from world import *

def sigma_ellipse(p, cov, n):
    '''Kalman Filterの誤差楕円'''
    eig_vals, eig_vec = np.linalg.eig(cov)
    ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180
    return Ellipse(p, width=2 * n * math.sqrt(eig_vals[0]), height=2 * n * math.sqrt(eig_vals[1]), angle=ang, fill=False, color="blue", alpha=0.5)

def matQ(noise):
    '''状態方程式の誤差共分散行列'''
    return noise

def matH(pose, ap_pos):
    '''ヤコビ行列'''
    mx, my = ap_pos
    mux, muy = pose
    q = (mux - mx)**2 + (muy - my)**2
    return np.array([[(mux - mx)/np.sqrt(q), (muy - my)/np.sqrt(q)]])

def observation_function(pose, landmark_pos):
    '''観測方程式の右辺'''
    dif = pose - landmark_pos
    return math.sqrt(dif[0]**2 + dif[1]**2)

def matR(distance_dev):
    return np.diag(np.array([distance_dev**2]))


class Kalman:
    def __init__(self, envmap, init_pose, motion_noise_stds=np.diag([100,100]), distance_dev_rate=300):
        self.belief = multivariate_normal(mean=init_pose, cov=np.diag([100, 100]))
        self.pose = self.belief.mean
        self.motion_noise_stds = motion_noise_stds
        self.map = envmap
        self.poses = [self.pose]
        self.distance_dev_rate = distance_dev_rate
        self.position = 0

    def motion_update(self):
        Q = matQ(self.motion_noise_stds)
        self.belief.cov = self.belief.cov + Q

    def observation_update(self, observation):
        for id in observation:
            z = observation[id][0]
            #self.distance_dev_rate = observation[id][1]
            ap_id = id
            ap_pos = None
            for ap in self.map.aps:
                if ap.id == ap_id:
                    ap_pos = ap.pos
                    break
            H = matH(self.belief.mean, ap_pos)
            estimated_z = observation_function(self.belief.mean, ap_pos)
            R = matR(distance_dev=self.distance_dev_rate)
            S = H.dot(self.belief.cov).dot(H.T) + R
            K = self.belief.cov.dot(H.T).dot(np.linalg.inv(S))
            self.belief.mean = self.belief.mean + (K.dot([z - estimated_z]))
            self.belief.cov = (np.eye(2) - K.dot(H)).dot(self.belief.cov)
            self.pose = self.belief.mean
        self.position += 1

    def draw(self, ax, elems):
        # xy平面上の誤差の3シグマ範囲 #
        e = sigma_ellipse(self.belief.mean[0:2], self.belief.cov[0:2, 0:2], 3)
        elems.append(ax.add_patch(e))

        # xy平面上の平均値 #
        x, y = self.belief.mean
        c = patches.Circle(xy=(x, y), radius=100, fill=True, color="red")
        elems.append(ax.add_patch(c))
        elems += ax.plot([self.pose[0], self.map.rps[(self.position-1)%8].pos[0]], [self.pose[1], self.map.rps[(self.position-1)%8].pos[1]], color="pink")

        if len(self.poses) >= 9:
            self.poses.pop(-9)
        self.poses.append(self.pose)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")

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

    init_pose = np.array([3330,8940]).T
    kf = Kalman(m, init_pose=init_pose)
    pedestrian = Pedestrian(pose=np.array([20,20]), estimator=kf, color="red")
    world.append(pedestrian)
    world.append(m)

    world.draw()