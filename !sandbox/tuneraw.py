import cv2, time
import numpy as np

filename = 'image1574623167.8772793'
oimg = cv2.imread('./rawimg/'+filename+'.jpg')
gray = cv2.cvtColor(oimg, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (15, 15), 0)
thresh = cv2.threshold(gray, 200, 210, cv2.THRESH_BINARY)[1]


contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
for c in contours:

    # x, y, w, h = cv2.boundingRect(c)
    # cv2.rectangle(oimg, (x, y), (x + w, y + h), (255, 255, 255), 2)

    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(oimg, [box], 0, (255, 255, 255), 2)
    area = cv2.contourArea(box)
    print(area)

    moments = cv2.moments(c)

    if moments["m00"] == 0:
        cv2.drawContours(oimg, [c], -1, (255, 0, 0), 2)
        continue

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])


    if 700 < area < 1400:
        cv2.drawContours(oimg, [c], -1, (0, 255, 0), 2)
    else:
        cv2.drawContours(oimg, [c], -1, (0, 0, 255), 2)

    # cv2.putText(oimg, str(area), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
    #             (100, 100, 100), 2)
    if rect[1][0] > 10:
        cv2.putText(oimg, str(round(rect[1][0]))+'-'+str(round(rect[1][1]))+'|'+str(area), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (10, 100, 100), 2)


# crop_img = oimg[top:(top + height), left:(left + width)]

cv2.imwrite("./out/image" + str(time.time()) + ".jpg", oimg)
cv2.imwrite("./out/image_bin" + str(time.time()) + ".jpg", thresh)
cv2.imshow('image', oimg)

# vis = np.concatenate((oimg, thresh), axis=1)
# cv2.imshow('image', oimg)
# cv2.imshow('image', thresh)

while True:
    if cv2.waitKey(1) & 0XFF == ord('q'):
        cv2.destroyAllWindows()
        break