import cv2
# https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        shape = "unknown"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.045 * peri, True) # orig: 0.04

        if len(approx) <= 5 or len(approx) > 3:
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            if w > 10 and h > 10:
                if ar >= 0.05 and ar <= 0.9:
                    shape = "rectangle"
                elif ar > 1.1:
                    shape = "rectangle"
        return shape