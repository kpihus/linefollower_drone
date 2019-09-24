from vision.eyes import Eyes
from wings.phoenix import Phoenix
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
    flight_commands = manager.LifoQueue()

    b = Phoenix(flight_params, flight_commands)
    eyes = Eyes(flight_params, flight_commands)
    info = Ears(flight_params)

    p1 = Process(target=info.connect)
    p1.start()

    p2 = Process(target=eyes.start_capture)
    p2.start()

    p3 = Process(target=b.connect)
    p3.start()

    p1.join()
    p2.join()
    p3.join()
