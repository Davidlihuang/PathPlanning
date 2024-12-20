# coding=utf-8
from __future__ import division
from __future__ import absolute_import
import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import env
import routingGraph

class Plotting(object):
    def __init__(self, rtGraph, xI, xG):
        self.xI, self.xG =xI,  xG
        self.env = env.Env()
        self.obs = self.env.obs_map()
        self.rtGraph = rtGraph

    def update_obs(self, obs):
        self.obs = obs

    def animation(self, path, visited, name, enable=False):
        if enable :
            self.plot_grid(name)
            self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def animation_lrta(self, path, visited, name):
        self.plot_grid(name)
        cl = self.color_list_2()
        path_combine = []

        for k in range(len(path)):
            self.plot_visited(visited[k], cl[k])
            plt.pause(0.2)
            self.plot_path(path[k])
            path_combine += path[k]
            plt.pause(0.2)
        if self.xI in path_combine:
            path_combine.remove(self.xI)
        self.plot_path(path_combine)
        plt.show()

    def animation_ara_star(self, path, visited, name):
        self.plot_grid(name)
        cl_v, cl_p = self.color_list()

        for k in range(len(path)):
            self.plot_visited(visited[k], cl_v[k])
            self.plot_path(path[k], cl_p[k], True)
            plt.pause(0.5)

        plt.show()

    def animation_bi_astar(self, path, v_fore, v_back, name):
        self.plot_grid(name)
        self.plot_visited_bi(v_fore, v_back)
        self.plot_path(path)
        plt.show()
    
    def plot_grid(self, name):
        plt.plot(self.xI[0], self.xI[1], u"bs")
        plt.plot(self.xG[0], self.xG[1], u"gs")
        for r in range(self.rtGraph.l):
            localObs = set()
            for x in range(self.rtGraph.w):
                for y in range(self.rtGraph.h):
                    if self.rtGraph.grid[x, y, r] == True:
                        localObs.add((x,y))
            obs_x = [x[0] for x in localObs]
            obs_y = [x[1] for x in localObs]
            plt.plot(obs_x, obs_y,  u"sk")
        plt.title(name)
        plt.axis(u"equal")

    def plot_visited(self, visited, cl=u'gray'):
        if self.xI in visited:
            visited.remove(self.xI)
        # print("visited: ", visited)
        if self.xG in visited:
            visited.remove(self.xG)

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker=u'o')
            plt.gcf().canvas.mpl_connect(u'key_release_event',
                                         lambda event: [exit(0) if event.key == u'escape' else None])
         
            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40
            #
            # length = 15
            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    def plot_path(self, path, cl=u'r', flag=False):
        cr =  self.color_list_2()
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        path_z = [path[i][2] for i in range(len(path))]
        for i in range(len(path)):
            plt.plot(path_x[i], path_y[i], u"o", color=cr[path_z[i]])
        plt.plot(self.xI[0], self.xI[1], u"bs")
        plt.plot(self.xG[0], self.xG[1], u"gs")

        plt.pause(0.01)

    def plot_visited_bi(self, v_fore, v_back):
        if self.xI in v_fore:
            v_fore.remove(self.xI)

        if self.xG in v_back:
            v_back.remove(self.xG)

        len_fore, len_back = len(v_fore), len(v_back)

        for k in range(max(len_fore, len_back)):
            if k < len_fore:
                plt.plot(v_fore[k][0], v_fore[k][1], linewidth=u'3', color=u'gray', marker=u'o')
            if k < len_back:
                plt.plot(v_back[k][0], v_back[k][1], linewidth=u'3', color=u'cornflowerblue', marker=u'o')

            plt.gcf().canvas.mpl_connect(u'key_release_event',
                                         lambda event: [exit(0) if event.key == u'escape' else None])

            if k % 10 == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    @staticmethod
    def color_list():
        cl_v = [u'silver',
                u'wheat',
                u'lightskyblue',
                u'royalblue',
                u'slategray']
        cl_p = [u'gray',
                u'orange',
                u'deepskyblue',
                u'red',
                u'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = [
              u'red', 
              u'blue',
              u'magenta',
              u'green', 
              u'deepskyblue',
               u'gold', 
               u'lime', 
               u'silver',
              u'steelblue',
              u'dimgray',
              u'cornflowerblue',
              u'dodgerblue',
              u'royalblue',
              u'plum',
              u'mediumslateblue',
              u'mediumpurple',
              u'blueviolet',
              ]
        return cl
