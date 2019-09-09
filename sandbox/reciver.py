import cv2
cap_receive = cv2.VideoCapture('udp://@:5600', cv2.CAP_GSTREAMER)

if not cap_receive.isOpened():
    print('VideoCapture not opened')
    exit(0)

while True:
    ret, frame = cap_receive.read()

    if not ret:
        print('empty frame')
        break

    cv2.imshow('receive', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# cap_receive.release()
cv2.destroyAllWindows()
