import cv2
from math import atan2, degrees, radians, sin, cos

GREEN = (0, 255, 0)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def calculateangle(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    angle = degrees(atan2(yDiff, xDiff))
    # if angle < 0:
    #     return 180 + angle
    return angle

def plot_point(start, angle, length):
    x1 = start[0]
    y1 = start[1]

    x2 = x1 + cos(radians(angle)) * length
    y2 = y1 + sin(radians(angle)) * length

    return (int(x2), int(y2))

def detect(c):
    shape = "unknown"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.045 * peri, True)  # orig: 0.04

    if len(approx) == 4:
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)

        if w > 20 and h > 20:
            if ar >= 0.05 and ar <= 0.9:
                shape = "rectangle"
            elif ar > 1.1:
                shape = "rectangle"

    return shape


img = cv2.imread('real_cross.png')
# img = cv2.flip(originalImage, 1)
rows, cols = img.shape[:2]
print("Cols" + str(cols))
print("Rows" + str(rows))

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (15, 15), 0)
thresh = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY)[1]

contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
cnt = contours[1]
shape = detect(cnt)
# for cnt in contours:
cv2.drawContours(img, cnt, -1, (0, 0, 255), 2)
moments = cv2.moments(cnt)
center = (int(cols / 2), int(rows /2))
fstart = (center[0] - 10, center[1] + 100)

# cv2.circle(img, center, 4, BLUE, -1)
# cv2.circle(img, fstart, 4, RED, -1)
# cv2.line(img, fstart, center, BLUE, 2)
# xDiff = center[0] - fstart[0]
# yDiff = center[1] - fstart[1]

# fangle = degrees(atan2(yDiff, xDiff)) * -1
# cv2.putText(img, "FA:"+str(round(fangle, 1)), (center[0] + 50, center[1] + 50), FONT, 0.5, YELLOW, 2)

if moments["m00"] != 0:# and shape == "rectangle":
    # continue

    # countour center
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])

    # then apply fitline() function
    [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L1, 0, 0.01, 0.01)

    vxa = vx
    vya = vy
    if vxa < 0 and vya < 0 or vxa > 0 and vya < 0:
        vxa = vxa * -1
        vya = vya * -1

    if vxa > 0 and vya > 0:
        vya = vya * -1

    if vxa < 0 and vya > 0:
        vxa = vxa * -1

    # vy = abs(vy)
    lefty = int((-x * vy / vx) + y)
    righty = int(((cols - x) * vy / vx) + y)

    point1 = (cols - 1, righty)
    point2 = (0, lefty)


    angle = degrees(atan2(vya, vxa))

    print('VX ' + str(vx))
    print('VY ' + str(vy))

    print("Angle: "+str(angle))

    # cv2.circle(img, point1, 7, (0, 255, 0), -1)  # green
    # cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)  # red

    cv2.circle(img, center, 7, RED, -1)

    # zero directiion

    zero_dir = plot_point(center, 0, 100)
    cv2.circle(img, zero_dir, 7, RED, -1)

    try:

        # Finally draw the line
        # cv2.line(img, point1, point2, RED, 2)  # neg angle, red



        cv2.putText(img, "A:"+str(round(angle, 1)), (int(cx) + 50, int(cy)), FONT, 0.5, GREEN, 2)


        cv2.putText(img, "ERROR: " +str(round(angle - fangle, 2)), (40, 40), FONT, 0.5, YELLOW, 2)

        # cv2.imshow('image', blurred)
        # cv2.imshow('tresh', thresh)
    except:
        pass

cv2.imshow('image', img)

while True:
    if cv2.waitKey(1) & 0XFF == ord('q'):
        cv2.destroyAllWindows()
        break
