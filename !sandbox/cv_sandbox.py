import cv2
from math import atan2, degrees


def calculateangle(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    angle = degrees(atan2(yDiff, xDiff))
    # if angle < 0:
    #     return 180 + angle
    return angle


# img = cv2.imread('box.png')

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, img = cap.read()
    if not ret:
        print('frame empty')
        break
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)
    thresh = cv2.threshold(blurred, 100, 200, cv2.THRESH_BINARY)[1]
    contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    cv2.drawContours(img, cnt, -1, (0, 0, 255), 1)
    moments = cv2.moments(cnt)

    if moments["m00"] == 0:
        break

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])

    # then apply fitline() function
    [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)

    # Now find two extreme points on the line to draw line
    lefty = int((-x * vy / vx) + y)
    righty = int(((gray.shape[1] - x) * vy / vx) + y)

    point1 = (gray.shape[1] - 1, righty)
    point2 = (0, lefty)

    angle = degrees(atan2(vy, vx))

    print(angle)
    # cv2.circle(img, point1, 7, (0, 255, 0), -1)  # green
    # cv2.circle(img, point2, 7, (0, 0, 255), -1)  # red

    try:
        cv2.putText(img, str(round(angle, 1)), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 0, 100), 2)

        # Finally draw the line
        cv2.line(img, point1, point2, 255, 2)

        cv2.imshow('image', img)
        # cv2.imshow('tresh', thresh)
    except:
        pass
    c = cv2.waitKey(1)
    if c == 27:
        break

cv2.destroyAllWindows()
