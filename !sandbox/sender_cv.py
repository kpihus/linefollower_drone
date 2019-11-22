import cv2
import base64
import zmq
import time

context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.connect('tcp://localhost:5555')

camera = cv2.VideoCapture(0)  # init the camera
n = 0
while True:
    n += 1

    try:
        grabbed, frame = camera.read()  # grab the current frame
        frame = cv2.resize(frame, (640, 480))  # resize the frame
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        now = time.time()
        encoded, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer)
        footage_socket.send(jpg_as_text)

    except KeyboardInterrupt:
        camera.release()
        cv2.destroyAllWindows()
        break
    print(str(n), str(time.time() - now))
    if n > 1000:
        exit()