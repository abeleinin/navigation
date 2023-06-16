#!/usr/bin/env python3

import math
import numpy as np

class Util:
    def __init__(self, circle, rect, bound):
      self.delta = 0.1
      self.obs_circle = circle
      self.obs_rectangle = rect
      self.obs_boundary = bound

    def get_obs_vertex(self):
      delta = self.delta
      obs_list = []

      for rect in self.obs_rectangle:
          ox = rect[0]
          oy = rect[1]
          w = rect[3]
          h = rect[4]
          vertex_list = [[ox - delta, oy - delta],
                          [ox + w + delta, oy - delta],
                          [ox + w + delta, oy + h + delta],
                          [ox - delta, oy + h + delta]]
          obs_list.append(vertex_list)

      return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        # div = np.dot(v2, v3)

        # if div == 0:
        #     return False

        # t1 = np.linalg.norm(np.cross(v2, v1)) / div # t1 = - t1 gives way better performance
        # t2 = np.dot(v1, v3) / div

        # algorithm based on https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
        
        n = [-v2[1],v2[0]]
        D = np.dot(n, a)
        num1 = np.dot(n,o) + D
        num2 = np.dot(n,v1) + D
        den = np.dot(n,d)

        if den == 0:
            return False
        t1 = - num1/den
        t2 = - num2/den
        
        if 0 < t1 < 1 and 0 < t2 < 1:
            shot = (o[0] + t1 * d[0], o[1] + t1 * d[1])
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        # d2 = np.dot(d, d)
        # delta = self.delta

        # if d2 == 0:
        #     return False

        # t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        # if 0 <= t <= 1:
        #     shot = rrt.Node((o[0] + t * d[0], o[1] + t * d[1]))
        #     if self.get_dist(shot, rrt.Node(a)) <= r + delta:
        #         return True

        # return False

        # based on https://www.cs.princeton.edu/courses/archive/fall00/cs426/lectures/raycast/sld013.htm
        
        r = r + self.delta # to take delta into account
        L = [a[0] - o[0], a[1] - o[1]]
        v = d / np.linalg.norm(d)
        T = np.dot(L,v)
        d2 = np.dot(L,L) - T*T 
        if T<0 or d2 > r*r:
            return False
        return True

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for vertex in obs_vertex:
            v1 = vertex[0]
            v2 = vertex[1]
            v3 = vertex[2]
            v4 = vertex[3]
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        for circle in self.obs_circle:
          x = circle[0] 
          y = circle[1]
          r = circle[3]
          if self.is_intersect_circle(o, d, [x, y], r):
            return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        for circle in self.obs_circle:
          x = circle[0]
          y = circle[1]
          r = circle[3]
          if math.hypot(node[0] - x, node[1] - y) <= r + delta:
            return True

        for rect in self.obs_rectangle:
          x = rect[0]
          y = rect[1]
          w = rect[3]
          h = rect[4]
          if 0 <= node[0] - (x - delta) <= w + 2 * delta \
              and 0 <= node[1] - (y - delta) <= h + 2 * delta:
            return True

        for bound in self.obs_boundary:
          x = bound[0]
          y = bound[1]
          w = bound[2]
          h = bound[3]
          if 0 <= node[0] - (x - delta) <= w + 2 * delta \
              and 0 <= node[1] - (y - delta) <= h + 2 * delta:
            return True

        return False

    @staticmethod
    def get_ray(start, end):
        orig = [start[0], start[1]]
        direc = [end[0] - start[0], end[1] - start[1]]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end[0] - start[0], end[1] - start[1])
