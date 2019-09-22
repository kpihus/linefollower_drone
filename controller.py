from vision.eyes import Eyes
# from wings.phoenix import Phoenix
from vision.eyes import Eyes
from ears.ears import Ears

from multiprocessing import Process
from multiprocessing.managers import BaseManager
from queue import LifoQueue


class QueueManager(BaseManager):
    pass


QueueManager.register('LifoQueue', LifoQueue)

if __name__ == "__main__":

    manager = QueueManager()
    manager.start()
    flight_params = manager.LifoQueue()

    # b = Phoenix(flight_params)
    eyes = Eyes(flight_params)
    info = Ears(flight_params)

    p1 = Process(target=info.connect)
    p1.start()

    p2 = Process(target=eyes.start_capture)
    p2.start()
    p1.join()
    p2.join()
