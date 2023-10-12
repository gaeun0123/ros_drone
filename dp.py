#!/usr/bin/env python3

from math import sqrt

def point_line_distance(point, start, end):
    x, y = point
    x1, y1 = start
    x2, y2 = end
    return abs((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1) / sqrt((y2 - y1)**2 + (x2 - x1)**2)

def douglas_peucker(points, epsilon):
    dmax = 0.0
    index = 0
    for i in range(1, len(points) - 1):
        d = point_line_distance(points[i], points[0], points[-1])
        if d > dmax:
            index = i
            dmax = d

    if dmax >= epsilon:
        results1 = douglas_peucker(points[:index + 1], epsilon)
        results2 = douglas_peucker(points[index:], epsilon)

        return results1[:-1] + results2
    else:
        return [points[0], points[-1]]