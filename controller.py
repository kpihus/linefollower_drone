from vision.eyes import Eyes
from wings.phoenix import Phoenix
from vision.eyes import Eyes
import queue, threading
from multiprocessing import Process


if __name__ == "__main__":
    q = queue.LifoQueue()

    b = Phoenix(q)
    e = Eyes(q)

    p1 = Process(target=b.connect)
    p2 = Process(target=e.start_capture)

    p1.start()
    p2.start()
