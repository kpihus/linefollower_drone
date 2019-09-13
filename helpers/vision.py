from math import atan2, degrees
from numpy import average
import cv2
import numpy as np


class Helpers:
    def __init__(self):
        pass

    def angleAvg(self, angle):
        vals = map(lambda a: a[2], angle)
        return average(vals)

    def rotateImage(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

    def averageLineDeg(self, lines):
        count = 0
        sum = 0
        for l in lines:
            count = count +1
            sum = sum + l.angle

        return sum / count

    def distance(self, pt_1, pt_2):
        pt_1 = np.array((pt_1[0], pt_1[1]))
        pt_2 = np.array((pt_2[0], pt_2[1]))
        return np.linalg.norm(pt_1 - pt_2)

    def closest_point_to(self, target, lines):
        global closest
        dist = 99999999
        for l in lines:
            if self.distance(target, l.guidepoint) <= dist:
                dist = self.distance(target, l.guidepoint)
                closest = l

        return closest
