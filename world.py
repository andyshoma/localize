#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.patches as patches
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import math


def get_observation(path):
    with open(path, encoding='utf-8', mode='r') as f:
        distances = []
        distance = {}
        for line in f:
            data = line.split(',')
            distanceMm = int(data[0])
            distance_dev = int(data[1])
            getID = int(data[4])

            if getID == 200372:
                distance[200372] = [distanceMm, distance_dev]
            elif getID == 200359:
                distance[200359] = [distanceMm, distance_dev]
            elif getID == 200358:
                distance[200358] = [distanceMm, distance_dev]

            if 200372 in distance and 200359 in distance and 200358 in distance:
                distances.append(distance)
                distance = {}

    return distances

def calibration(obs):
    for ob in obs:
        ob[200372][0] -= 3481
        ob[200359][0] -= 4860
        ob[200358][0] -= 3109

        if ob[200372][0] < 0:
            ob[200372][0] = 100
        if ob[200359][0] < 0:
            ob[200359][0] = 100
        if ob[200358][0] < 0:
            ob[200358][0] = 100

    return obs

def observation_error(envmap, observation, i):
    errors = []
    for id in observation:
        z = observation[id][0]
        ap_pos = None
        for ap in envmap.aps:
            if ap.id == id: ap_pos = ap.pos

        distance_true = observation_function(envmap.rps[(i-1)%8].pos, ap_pos)
        error = int(z - distance_true)
        errors.append(str(id) + " : " + str(error))

    return errors

def observation_function(pose, landmark_pos):
    '''観測方程式の右辺'''
    dif = pose - landmark_pos
    return math.sqrt(dif[0]**2 + dif[1]**2)


class World:
    def __init__(self, time_span, time_interval, debug=False):
        self.objects = []
        self.debug = debug
        self.time_span = time_span
        self.time_interval = time_interval

    def append(self, obj):
        self.objects.append(obj)

    def draw(self):
        fig = plt.figure(figsize=(4,4))
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        ax.set_xlim(-1000, 13000)
        ax.set_ylim(-1000, 18000)
        ax.set_xlabel("X", fontsize=10)
        ax.set_ylabel("Y", fontsize=10)
        elems = []

        if self.debug:
            for i in range(int(self.time_span / self.time_interval)): self.one_step(i, elems, ax)
        else:
            self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems, ax), frames=int(self.time_span / self.time_interval) + 1, interval=int(self.time_interval * 1000), repeat=False)

            #self.ani.save('kf_true.gif', writer='pillow')
            plt.show()

    def one_step(self, i, elems, ax):
        while elems: elems.pop().remove()
        time_str = "t = %d" % (i+1)
        elems.append(ax.text(-4.4, 4.5, time_str, fontsize=10))
        for obj in self.objects:
            obj.draw(ax, elems)
            if hasattr(obj, "one_step"): obj.one_step(self.time_interval, i)


class Pedestrian:
    def __init__(self, pose, estimator=None, sensor=None, color="black"):
        self.pose = pose
        self.r = 100
        self.color = color
        self.poses = [pose]
        self.sensor = sensor
        self.estimator = estimator
        self.observations = get_observation("/Users/andy/Wifi_rtt/localize0117/positionA.csv")
        self.observations = calibration(self.observations)
        self.errors = [0, 0, 0]

    def draw(self, ax, elems):  ### call_agent_draw
        self.poses.append(self.pose)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")
        elems.append(ax.text(-4.4, 17000, str(self.errors[0]), fontsize=8))
        elems.append(ax.text(-4.4, 16000, str(self.errors[1]), fontsize=8))
        elems.append(ax.text(-4.4, 15000, str(self.errors[2]), fontsize=8))
        if self.sensor and len(self.poses) > 1:
            self.sensor.draw(ax, elems, self.poses[-2])
        if self.estimator and hasattr(self.estimator, "draw"):
            self.estimator.draw(ax, elems)

    def one_step(self, time_interval, i):
        self.estimator.motion_update()
        self.estimator.observation_update(self.observations[i])
        self.errors = observation_error(self.estimator.map, self.observations[i], i)


class AccessPoint:
    def __init__(self, x, y, id):
        self.pos = np.array([x, y]).T
        self.id = id

    def draw(self, ax, elems):
        c = ax.scatter(self.pos[0], self.pos[1], s=100, marker="*", label="ap", color="orange")
        elems.append(c)
        elems.append(ax.text(self.pos[0], self.pos[1], str(self.id), fontsize=10))


class ReferencePoint:
    def __init__(self, x, y, id):
        self.pos = np.array([x, y]).T
        self.id = id

    def draw(self, ax, elems):
        c = patches.Circle(xy=(self.pos[0], self.pos[1]), radius=100, fill=True, color="blue")
        elems.append(ax.add_patch(c))
        elems.append(ax.text(self.pos[0], self.pos[1], str(self.id), fontsize=10))


class Obstacle:
    def __init__(self, x, y, width, height):
        self.pos = np.array([x, y]).T
        self.width = width
        self.height = height

    def draw(self, ax, elems):
        rec = patches.Rectangle(xy=(self.pos[0], self.pos[1]), width=self.width, height=self.height, color="green", fill=False)
        elems.append(ax.add_patch(rec))

class WifiFtm:
    def __init__(self, env_map):
        self.map = env_map
        self.observations = get_observation("/Users/andy/Wifi_rtt/localize0117/positionA.csv")
        self.observations = calibration(self.observations)
        self.lastdata = []

    def data(self, i):
        self.lastdata = self.observations[i]
        return self.observations[i]

    def draw(self, ax, elems, pose):
        for lm in self.lastdata:
            x, y = pose
            distance = self.lastdata[lm]
            lx = x + distance


class Map:
    def __init__(self):
        self.aps = []
        self.rps = []
        self.obs = []

    def appendAp(self, ap):
        self.aps.append(ap)

    def appendRp(self, rp):
        self.rps.append(rp)

    def appendObstacle(self, ob):
        self.obs.append(ob)

    def draw(self, ax, elems):
        for lm in self.aps: lm.draw(ax, elems)
        for lm in self.rps: lm.draw(ax, elems)
        for lm in self.obs: lm.draw(ax, elems)



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