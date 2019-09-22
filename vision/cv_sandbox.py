import cv2
import math

def plot_point(point, angle, length):
    angle = angle - 90

    # unpack the first point
    startx = point[0]
    starty = point[1]

    x1 = startx
    y1 = starty
    x2 = x1 + math.cos(math.radians(angle)) * length
    y2 = y1 + math.sin(math.radians(angle)) * length

    return [(int(x1), int(y1)), (int(x2), int(y2))]


frame = cv2.imread('dummy_test.png')

rows, cols = frame.shape[:2] # frame dimensions
img_center = (int(cols / 2), int(rows / 2))
print(img_center)
cv2.circle(frame, img_center, 7, (255, 100, 100), -1) # Image center point


point1 = plot_point(img_center, (270), 100)



cv2.circle(frame, point1[1], 7, (255, 100, 255), -1)

cv2.imshow('image', frame)

while True:
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break

cv2.destroyAllWindows()
