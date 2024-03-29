from math import atan2, degrees
from helpers.vision import Helpers
import math


class Line:
    def __init__(self, guidepoint, p1, p2, imgcenter):
        self.guidepoint = guidepoint
        self.p1 = p1
        self.p2 = p2
        self.angle = self.calculateangle(p1, p2)
        self.centerdistance = self.calculatecenterdistance(imgcenter, guidepoint)
        self.imgcenter = imgcenter
        pass

    def calculateangle(self, p1, p2):
        xDiff = p2[0] - p1[0]
        yDiff = p2[1] - p1[1]
        angle = degrees(atan2(yDiff, xDiff))
        # if angle < 0:
        #     return 180 + angle
        return angle

    def calculatecenterdistance(self, center, guidepoint):
        h = Helpers()
        return h.distance(center, guidepoint)

    def plot_point(self, point, angle, length):
        # angle = angle - 90

        # unpack the first point
        startx = point[0]
        starty = point[1]

        x1 = startx
        y1 = starty
        x2 = x1 + math.cos(math.radians(angle)) * length
        y2 = y1 + math.sin(math.radians(angle)) * length

        return [(int(x1), int(y1)), (int(x2), int(y2))]

    def line_intersection(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])  # Typo was here

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            raise Exception('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return int(x), int(y)
