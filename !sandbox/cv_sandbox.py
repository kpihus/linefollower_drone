import cv2
from math import atan2, degrees


def calculateangle(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    angle = degrees(atan2(yDiff, xDiff))
    # if angle < 0:
    #     return 180 + angle
    return angle


img = cv2.imread('box.png')

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cnt = contours[2]
moments = cv2.moments(cnt)

cx = int(moments["m10"] / moments["m00"])
cy = int(moments["m01"] / moments["m00"])

# then apply fitline() function
[vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)

# Now find two extreme points on the line to draw line
lefty = int((-x * vy / vx) + y)
righty = int(((gray.shape[1] - x) * vy / vx) + y)

point1 = (gray.shape[1] - 1, righty)
point2 = (0, lefty)

angle = calculateangle(point1, point2) + 90

cv2.circle(img, point1, 7, (0, 255, 0), -1)  # green
cv2.circle(img, point2, 7, (0, 0, 255), -1)  # red

cv2.putText(img, str(round(angle, 1)), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 100), 2)

# Finally draw the line
cv2.line(img, point1, point2, 255, 2)

cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
